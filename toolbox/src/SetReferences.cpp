/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "SetReferences.h"
#include "BlockInformation.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <yarp/dev/ControlBoardInterfaces.h>

#include <cmath>

using namespace wbt;

const std::string SetReferences::ClassName = "SetReferences";

const unsigned PARAM_IDX_BIAS = WBBlock::NumberOfParameters - 1;
const unsigned PARAM_IDX_CTRL_TYPE = PARAM_IDX_BIAS + 1;
const unsigned PARAM_IDX_REF_SPEED = PARAM_IDX_BIAS + 2;

const std::vector<double> SetReferences::rad2deg(const double* buffer, const unsigned width)
{
    const double Rad2Deg = 180.0 / M_PI;

    std::vector<double> vectorDeg(width);

    for (auto i = 0; i < width; ++i) {
        vectorDeg[i] = buffer[i] * Rad2Deg;
    }

    return vectorDeg;
}

unsigned SetReferences::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 2;
}

bool SetReferences::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata paramMD_ctrlType(
        ParameterType::STRING, PARAM_IDX_CTRL_TYPE, 1, 1, "CtrlType");
    ParameterMetadata paramMD_refSpeed(
        ParameterType::DOUBLE, PARAM_IDX_REF_SPEED, 1, 1, "RefSpeed");

    bool ok = true;
    ok = ok && blockInfo->addParameterMetadata(paramMD_ctrlType);
    ok = ok && blockInfo->addParameterMetadata(paramMD_refSpeed);

    if (!ok) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool SetReferences::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // Memory allocation / Saving data not allowed here

    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // INPUTS
    // ======
    //
    // 1) Joint refereces (1xDoFs vector)
    //

    // Number of inputs
    if (!blockInfo->setNumberOfInputPorts(1)) {
        wbtError << "Failed to configure the number of input ports.";
        return false;
    }

    // Get the DoFs
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }
    const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();

    // Size and type
    bool success = blockInfo->setInputPortVectorSize(0, dofs);
    blockInfo->setInputPortType(0, DataType::DOUBLE);

    if (!success) {
        wbtError << "Failed to configure input ports.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // No outputs
    //

    if (!blockInfo->setNumberOfOutputPorts(0)) {
        wbtError << "Failed to configure the number of output ports.";
        return false;
    }

    return true;
}

bool SetReferences::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // INPUT PARAMETERS
    // ================

    if (!parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    std::string controlType;
    if (!m_parameters.getParameter("CtrlType", controlType)) {
        wbtError << "Could not read control type parameter.";
        return false;
    }

    if (!m_parameters.getParameter("RefSpeed", m_refSpeed)) {
        wbtError << "Could not read reference speed parameter.";
        return false;
    }

    // PRIVATE MEMBERS
    // ===============

    // Get the DoFs
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }
    const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();

    // Retain the ControlBoardRemapper
    if (!robotInterface->retainRemoteControlBoardRemapper()) {
        wbtError << "Couldn't retain the RemoteControlBoardRemapper.";
        return false;
    }

    // Initialize the size of std::vectors
    m_controlModes.assign(dofs, VOCAB_CM_UNKNOWN);

    // IControlMode.h
    if (controlType == "Position") {
        m_controlModes.assign(dofs, VOCAB_CM_POSITION);
    }
    else if (controlType == "Position Direct") {
        m_controlModes.assign(dofs, VOCAB_CM_POSITION_DIRECT);
    }
    else if (controlType == "Velocity") {
        m_controlModes.assign(dofs, VOCAB_CM_VELOCITY);
    }
    else if (controlType == "Torque") {
        m_controlModes.assign(dofs, VOCAB_CM_TORQUE);
    }
    else if (controlType == "PWM") {
        m_controlModes.assign(dofs, VOCAB_CM_PWM);
    }
    else if (controlType == "Current") {
        m_controlModes.assign(dofs, VOCAB_CM_CURRENT);
    }
    else {
        wbtError << "Control Mode not supported.";
        return false;
    }

    // The Position mode is used to set a discrete reference, and then the yarp interface
    // is responsible to generate a trajectory to reach this setpoint.
    // The generated trajectory takes an additional parameter: the speed.
    // If not properly initialized, this contol mode does not work as expected.
    if (controlType == "Position") {
        // Get the interface
        yarp::dev::IPositionControl* interface = nullptr;
        if (!robotInterface->getInterface(interface) || !interface) {
            wbtError << "Failed to get IPositionControl interface.";
            return false;
        }
        std::vector<double> speedInitalization(dofs, m_refSpeed);
        // Set the references
        if (!interface->setRefSpeeds(speedInitalization.data())) {
            wbtError << "Failed to initialize speed references.";
            return false;
        }
    }

    m_resetControlMode = true;
    return true;
}

bool SetReferences::terminate(const BlockInformation* blockInfo)
{
    using namespace yarp::dev;
    bool ok = true;

    // Get the RobotInterface
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "Couldn't get RobotInterface.";
        return false;
    }

    // Get the DoFs
    const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();

    // Get the IControlMode2 interface
    IControlMode2* icmd2 = nullptr;
    ok = ok && robotInterface->getInterface(icmd2);
    if (!ok || !icmd2) {
        wbtError << "Failed to get the IControlMode2 interface.";
        // Don't return false here. WBBlock::terminate must be called in any case
    }

    // Set  all the controlledJoints VOCAB_CM_POSITION
    m_controlModes.assign(dofs, VOCAB_CM_POSITION);

    ok = ok && icmd2->setControlModes(m_controlModes.data());
    if (!ok) {
        wbtError << "Failed to set control mode.";
        // Don't return false here. WBBlock::terminate must be called in any case
    }

    // Release the RemoteControlBoardRemapper
    ok = ok && robotInterface->releaseRemoteControlBoardRemapper();
    if (!ok) {
        wbtError << "Failed to release the RemoteControlBoardRemapper.";
        // Don't return false here. WBBlock::terminate must be called in any case
    }

    return ok && WBBlock::terminate(blockInfo);
}

bool SetReferences::initializeInitialConditions(const BlockInformation* /*blockInfo*/)
{
    // This function is called when a subsystem with Enable / Disable support is used.
    // In this case all the blocks in this subsystem are configured and initialize,
    // but if they are disabled, output() is not called.
    // This initializeInitialConditions method is called when the block is enabled,
    // and in this case the control mode should be set.
    //
    // It is worth noting that this toolbox disables parameters to be tunable for
    // all the blocks.

    // Set again the control mode on the first output() call after the new enabling
    // of the block
    m_resetControlMode = true;

    return true;
}

bool SetReferences::output(const BlockInformation* blockInfo)
{
    using namespace yarp::dev;

    // Get the RobotInterface
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "Couldn't get RobotInterface.";
        return false;
    }

    // Set the control mode at the first run
    if (m_resetControlMode) {
        m_resetControlMode = false;
        // Get the interface
        IControlMode2* icmd2 = nullptr;
        if (!robotInterface->getInterface(icmd2) || !icmd2) {
            wbtError << "Failed to get the IControlMode2 interface.";
            return false;
        }
        // Set the control mode to all the controlledJoints
        if (!icmd2->setControlModes(m_controlModes.data())) {
            wbtError << "Failed to set control mode.";
            return false;
        }
    }

    // Get the signal
    const Signal references = blockInfo->getInputPortSignal(0);
    const unsigned signalWidth = references.getWidth();

    if (!references.isValid()) {
        wbtError << "Input signal not valid.";
        return false;
    }

    double* bufferReferences = references.getBuffer<double>();
    if (!bufferReferences) {
        wbtError << "Failed to get the buffer containing references.";
        return false;
    }

    bool ok = false;
    // TODO: here only the first element is checked
    switch (m_controlModes.front()) {
        case VOCAB_CM_UNKNOWN:
            wbtError << "Control mode has not been successfully set.";
            return false;
            break;
        case VOCAB_CM_POSITION: {
            // Get the interface
            IPositionControl* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get IPositionControl interface.";
                return false;
            }
            // Convert from rad to deg
            auto referencesDeg = rad2deg(bufferReferences, signalWidth);
            // Set the references
            ok = interface->positionMove(referencesDeg.data());
            break;
        }
        case VOCAB_CM_POSITION_DIRECT: {
            // Get the interface
            IPositionDirect* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get IPositionDirect interface.";
                return false;
            }
            // Convert from rad to deg
            auto referencesDeg = rad2deg(bufferReferences, signalWidth);
            // Set the references
            ok = interface->setPositions(referencesDeg.data());
            break;
        }
        case VOCAB_CM_VELOCITY: {
            // Get the interface
            IVelocityControl* interface = nullptr;
            if (!robotInterface->getInterface(interface) || !interface) {
                wbtError << "Failed to get IVelocityControl interface.";
                return false;
            }
            // Convert from rad to deg
            auto referencesDeg = rad2deg(bufferReferences, signalWidth);
            // Set the references
            ok = interface->velocityMove(referencesDeg.data());
            break;
        }
        case VOCAB_CM_TORQUE: {
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
        case VOCAB_CM_PWM: {
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
        case VOCAB_CM_CURRENT: {
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
    }

    if (!ok) {
        wbtError << "Failed to set references.";
        return false;
    }

    return true;
}
