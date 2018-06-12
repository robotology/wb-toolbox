/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "SetReferences.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>

#include <cmath>
#include <ostream>
#include <tuple>
#include <vector>

using namespace wbt;
const std::string SetReferences::ClassName = "SetReferences";

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = WBBlock::NumberOfParameters - 1,
    CtrlType,
    RefSpeed
};

enum InputIndex
{
    References = 0,
};

// BLOCK PIMPL
// ===========

class SetReferences::impl
{
public:
    enum class ReferenceType
    {
        // Joint Measurements
        JOINT_POSITION,
        JOINT_POSITION_DIRECT,
        JOINT_VELOCITY,
        JOINT_TORQUE,
        // Motor measurements
        MOTOR_CURRENT,
        MOTOR_PWM,
        MOTOR_BEMF,
        MOTOR_KTAU,
    };

    std::vector<int> controlModes;
    bool resetControlMode = true;

    double refSpeed;
    std::vector<double> defaultRefSpeed;

    std::vector<yarp::dev::MotorTorqueParameters> motorParams;
    std::vector<yarp::dev::MotorTorqueParameters> defaultMotorParams;

    ReferenceType referenceType;

    std::vector<double> degBufferReferences;
    std::vector<double> previousReferenceVocabCmPosition;

    static void rad2deg(const double* buffer, const unsigned width, std::vector<double>& bufferDeg)
    {
        bufferDeg.resize(width);
        constexpr double Rad2Deg = 180.0 / M_PI;

        for (unsigned i = 0; i < width; ++i) {
            bufferDeg[i] = buffer[i] * Rad2Deg;
        }
    }
};

// BLOCK CLASS
// ===========

SetReferences::SetReferences()
    : pImpl{new impl()}
{}

unsigned SetReferences::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 2;
}

bool SetReferences::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> metadata{
        {ParameterType::STRING, ParamIndex::CtrlType, 1, 1, "CtrlType"},
        {ParameterType::DOUBLE, ParamIndex::RefSpeed, 1, 1, "RefSpeed"}};

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            wbtError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool SetReferences::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // Get the DoFs
    const int dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    // INPUTS
    // ======
    //
    // 1) Joint refereces (1xDoFs vector)
    //
    // OUTPUTS
    // =======
    //
    // No outputs
    //

    const bool ok = blockInfo->setIOPortsData({
        {
            // Inputs
            std::make_tuple(InputIndex::References, std::vector<int>{dofs}, DataType::DOUBLE),
        },
        {
            // Outputs
        },
    });

    if (!ok) {
        wbtError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool SetReferences::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // Get the DoFs
    const auto dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    // PARAMETERS
    // ==========

    if (!SetReferences::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    std::string controlType;
    if (!m_parameters.getParameter("CtrlType", controlType)) {
        wbtError << "Could not read control type parameter.";
        return false;
    }

    if (!m_parameters.getParameter("RefSpeed", pImpl->refSpeed)) {
        wbtError << "Could not read reference speed parameter.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    // Initialize the size of std::vectors
    pImpl->controlModes.assign(dofs, VOCAB_CM_UNKNOWN);

    // From IControlMode.h.
    //
    // Joint Control Modes:
    if (controlType == "Position") {
        pImpl->referenceType = impl::ReferenceType::JOINT_POSITION;
        pImpl->controlModes.assign(dofs, VOCAB_CM_POSITION);
    }
    else if (controlType == "Position Direct") {
        pImpl->referenceType = impl::ReferenceType::JOINT_POSITION_DIRECT;
        pImpl->controlModes.assign(dofs, VOCAB_CM_POSITION_DIRECT);
    }
    else if (controlType == "Velocity") {
        pImpl->referenceType = impl::ReferenceType::JOINT_VELOCITY;
        pImpl->controlModes.assign(dofs, VOCAB_CM_VELOCITY);
    }
    else if (controlType == "Torque") {
        pImpl->referenceType = impl::ReferenceType::JOINT_TORQUE;
        pImpl->controlModes.assign(dofs, VOCAB_CM_TORQUE);
    }
    // Motor Control Modes:
    else if (controlType == "PWM") {
        pImpl->referenceType = impl::ReferenceType::MOTOR_PWM;
        pImpl->controlModes.assign(dofs, VOCAB_CM_PWM);
    }
    else if (controlType == "Current") {
        pImpl->referenceType = impl::ReferenceType::MOTOR_CURRENT;
        pImpl->controlModes.assign(dofs, VOCAB_CM_CURRENT);
    }
    else if (controlType == "Back EMF") {
        pImpl->referenceType = impl::ReferenceType::MOTOR_BEMF;
        pImpl->controlModes.assign(dofs, VOCAB_CM_TORQUE);
    }
    else if (controlType == "Torque Constant") {
        pImpl->referenceType = impl::ReferenceType::MOTOR_KTAU;
        pImpl->controlModes.assign(dofs, VOCAB_CM_TORQUE);
    }
    else {
        wbtError << "Control Mode not supported.";
        return false;
    }

    // The Position mode is used to set a discrete reference, and then the yarp interface
    // is responsible to generate a trajectory to reach this setpoint.
    // The generated trajectory takes an additional parameter: the speed.
    // If not properly initialized, this contol mode does not work as expected.
    if (pImpl->referenceType == impl::ReferenceType::JOINT_POSITION) {
        // Convert the reference speed from radians to degrees
        pImpl->refSpeed *= 180.0 / M_PI;

        // Get the interface
        yarp::dev::IPositionControl* interface = nullptr;
        if (!getRobotInterface()->getInterface(interface) || !interface) {
            wbtError << "Failed to get IPositionControl interface.";
            return false;
        }

        // Store the default reference speeds
        pImpl->defaultRefSpeed.resize(dofs);
        if (!interface->getRefSpeeds(pImpl->defaultRefSpeed.data())) {
            wbtError << "Failed to get default reference speed.";
            return false;
        }

        // Set the new reference speeds
        std::vector<double> speedInitalization(dofs, pImpl->refSpeed);
        if (!interface->setRefSpeeds(speedInitalization.data())) {
            wbtError << "Failed to initialize reference speed.";
            return false;
        }

        // In this control mode, the reference should be sent only when it changes.
        // This vector caches the target joint positions.
        pImpl->previousReferenceVocabCmPosition.resize(dofs, 0);
    }
    else if (pImpl->referenceType == impl::ReferenceType::MOTOR_BEMF
             || pImpl->referenceType == impl::ReferenceType::MOTOR_KTAU) {
        // Get the interface
        yarp::dev::ITorqueControl* iTorqueControl = nullptr;
        if (!getRobotInterface()->getInterface(iTorqueControl) || !iTorqueControl) {
            wbtError << "Failed to get ITorqueControl interface.";
            return false;
        }
        // Resize the vectors
        pImpl->motorParams.resize(dofs);
        pImpl->defaultMotorParams.resize(dofs);
        // Get the default values
        for (unsigned m = 0; m < dofs; ++m) {
            if (!iTorqueControl->getMotorTorqueParams(m, &pImpl->defaultMotorParams[m])) {
                wbtError << "Failed to get motor torque parameters.";
                return false;
            }
        }
        pImpl->motorParams = pImpl->defaultMotorParams;
    }

    pImpl->resetControlMode = true;
    return true;
}

bool SetReferences::terminate(const BlockInformation* blockInfo)
{
    using namespace yarp::dev;

    // Get the RobotInterface and the DoFs
    const auto robotInterface = getRobotInterface();
    const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();

    // Get the IControlMode interface
    IControlMode* icmd = nullptr;
    if (!robotInterface->getInterface(icmd) || !icmd) {
        wbtError << "Failed to get the IControlMode interface.";
        return false;
    }

    // In Position mode restore the default reference speeds
    if (pImpl->referenceType == impl::ReferenceType::JOINT_POSITION) {
        // Get the interface
        yarp::dev::IPositionControl* interface = nullptr;
        if (!robotInterface->getInterface(interface) || !interface) {
            wbtError << "Failed to get IPositionControl interface.";
            return false;
        }
        // Restore default reference speeds
        if (!interface->setRefSpeeds(pImpl->defaultRefSpeed.data())) {
            wbtError << "Failed to restore default reference speed.";
            return false;
        }
    }
    // In MOTOR_BEMF or MOTOR_KTAU restore the default values
    else if (pImpl->referenceType == impl::ReferenceType::MOTOR_BEMF
             || pImpl->referenceType == impl::ReferenceType::MOTOR_KTAU) {
        // Get the interface
        ITorqueControl* interface = nullptr;
        if (!robotInterface->getInterface(interface) || !interface) {
            wbtError << "Failed to get ICurrentControl interface.";
            return false;
        }
        for (unsigned m = 0; m < dofs; ++m) {
            if (!interface->setMotorTorqueParams(m, pImpl->defaultMotorParams[m])) {
                wbtError << "Failed to restore default motor parameters.";
                return false;
            }
        }
    }
    else {
        // Set all the controlledJoints VOCAB_CM_POSITION
        pImpl->controlModes.assign(dofs, VOCAB_CM_POSITION);

        if (!icmd->setControlModes(pImpl->controlModes.data())) {
            wbtError << "Failed to restore default position control mode.";
            return false;
        }
    }

    return WBBlock::terminate(blockInfo);
}

bool SetReferences::initializeInitialConditions(const BlockInformation* /*blockInfo*/)
{
    // This function is called when a subsystem with Enable / Disable support is used.
    // In this case all the blocks in this subsystem are configured and initialize,
    // but if they are disabled, output() is not called.
    // This initializeInitialConditions method is called when the block is enabled,
    // and in this case the control mode should be set.
    //
    // It is worth noting that this toolbox does not support tunable parameters.

    // Set again the control mode on the first output() call after the new enabling
    // of the block
    pImpl->resetControlMode = true;

    return true;
}

bool SetReferences::output(const BlockInformation* blockInfo)
{
    using namespace yarp::dev;

    // Get the RobotInterface
    const auto robotInterface = getRobotInterface();

    // Set the control mode at the first run
    if (pImpl->resetControlMode) {
        pImpl->resetControlMode = false;
        // Get the interface
        IControlMode* icmd = nullptr;
        if (!robotInterface->getInterface(icmd) || !icmd) {
            wbtError << "Failed to get the IControlMode interface.";
            return false;
        }
        // Set the control mode to all the controlledJoints
        if (!icmd->setControlModes(pImpl->controlModes.data())) {
            wbtError << "Failed to set control mode.";
            return false;
        }
    }

    // Get the signal
    const Signal references = blockInfo->getInputPortSignal(InputIndex::References);
    const unsigned signalWidth = references.getWidth();

    if (!references.isValid()) {
        wbtError << "Input signal not valid.";
        return false;
    }

    const double* bufferReferences = references.getBuffer<double>();
    if (!bufferReferences) {
        wbtError << "Failed to get the buffer containing references.";
        return false;
    }

    if (pImpl->controlModes.front() == VOCAB_CM_UNKNOWN) {
        wbtError << "Control mode has not been successfully set.";
        return false;
    }

    bool ok = false;
    switch (pImpl->referenceType) {
        case impl::ReferenceType::JOINT_POSITION: {
            // Do not update the position reference if it didn't change.
            // This would generate a warning.
            const auto oldReference = pImpl->previousReferenceVocabCmPosition;
            const auto& dofs = robotInterface->getConfiguration().getNumberOfDoFs();
            pImpl->previousReferenceVocabCmPosition.assign(bufferReferences,
                                                           bufferReferences + dofs);
            if (oldReference == pImpl->previousReferenceVocabCmPosition) {
                ok = true;
                break;
            }
            // Get the interface
            IPositionControl* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get IPositionControl interface.";
                return false;
            }
            // Convert from rad to deg
            SetReferences::impl::rad2deg(bufferReferences, signalWidth, pImpl->degBufferReferences);
            // Set the references
            ok = interface->positionMove(pImpl->degBufferReferences.data());
            break;
        }
        case impl::ReferenceType::JOINT_POSITION_DIRECT: {
            // Get the interface
            IPositionDirect* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get IPositionDirect interface.";
                return false;
            }
            // Convert from rad to deg
            SetReferences::impl::rad2deg(bufferReferences, signalWidth, pImpl->degBufferReferences);
            // Set the references
            ok = interface->setPositions(pImpl->degBufferReferences.data());
            break;
        }
        case impl::ReferenceType::JOINT_VELOCITY: {
            // Get the interface
            IVelocityControl* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get IVelocityControl interface.";
                return false;
            }
            // Convert from rad to deg
            SetReferences::impl::rad2deg(bufferReferences, signalWidth, pImpl->degBufferReferences);
            // Set the references
            ok = interface->velocityMove(pImpl->degBufferReferences.data());
            break;
        }
        case impl::ReferenceType::JOINT_TORQUE: {
            // Get the interface
            ITorqueControl* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get ITorqueControl interface.";
                return false;
            }
            // Set the references
            ok = interface->setRefTorques(bufferReferences);
            break;
        }
        case impl::ReferenceType::MOTOR_PWM: {
            // Get the interface
            IPWMControl* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get IPWMControl interface.";
                return false;
            }
            // Set the references
            ok = interface->setRefDutyCycles(bufferReferences);
            break;
        }
        case impl::ReferenceType::MOTOR_CURRENT: {
            // Get the interface
            ICurrentControl* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get ICurrentControl interface.";
                return false;
            }
            // Set the references
            ok = interface->setRefCurrents(bufferReferences);
            break;
        }
        case impl::ReferenceType::MOTOR_BEMF: {
            // Get the interface
            ITorqueControl* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get ITorqueControl interface.";
                return false;
            }
            // Get the DoFs
            const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();
            for (unsigned m = 0; m < dofs; ++m) {
                pImpl->motorParams[m].bemf = bufferReferences[m];
                ok = interface->setMotorTorqueParams(m, pImpl->motorParams[m]);
                if (!ok) {
                    break;
                }
            }
            break;
        }
        case impl::ReferenceType::MOTOR_KTAU: {
            // Get the interface
            ITorqueControl* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get ITorqueControl interface.";
                return false;
            }
            // Get the DoFs
            const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();
            for (unsigned m = 0; m < dofs; ++m) {
                pImpl->motorParams[m].ktau = bufferReferences[m];
                ok = interface->setMotorTorqueParams(m, pImpl->motorParams[m]);
                if (!ok) {
                    break;
                }
            }
            break;
        }
    }

    if (!ok) {
        wbtError << "Failed to set references.";
        return false;
    }

    return true;
}
