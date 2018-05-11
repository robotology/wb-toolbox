/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "GetMeasurement.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncoders.h>
#include <yarp/dev/IMotorEncoders.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/ITorqueControl.h>

#include <cmath>
#include <ostream>
#include <tuple>
#include <vector>

using namespace wbt;
const std::string GetMeasurement::ClassName = "GetMeasurement";

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = WBBlock::NumberOfParameters - 1,
    MeasType
};

enum OutputIndex
{
    Measurement = 0,
};

// BLOCK PIMPL
// ===========

class GetMeasurement::impl
{
public:
    enum class MeasuredType
    {
        // Joint Measurements
        JOINT_POS,
        JOINT_VEL,
        JOINT_ACC,
        JOINT_TORQUE,
        // Motor measurements
        MOTOR_POS,
        MOTOR_VEL,
        MOTOR_ACC,
        MOTOR_CURRENT,
        MOTOR_PWM
    };

    std::vector<double> measurement;
    MeasuredType measuredType;

    static void deg2rad(std::vector<double>& v)
    {
        const double Deg2Rad = M_PI / 180.0;
        for (auto& element : v) {
            element *= Deg2Rad;
        }
    }
};

// BLOCK CLASS
// ===========

GetMeasurement::GetMeasurement()
    : pImpl{new impl()}
{}

unsigned GetMeasurement::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool GetMeasurement::parseParameters(BlockInformation* blockInfo)
{
    const ParameterMetadata measTypeMetadata(
        ParameterType::STRING, ParamIndex::MeasType, 1, 1, "MeasuredType");

    if (!blockInfo->addParameterMetadata(measTypeMetadata)) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool GetMeasurement::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // Get the DoFs
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }
    const int dofs = robotInterface->getConfiguration().getNumberOfDoFs();

    // INPUTS
    // ======
    //
    // No inputs
    //
    // OUTPUTS
    // =======
    //
    // 1) Vector with the information asked (1xDoFs)
    //

    const bool ok = blockInfo->setIOPortsData({
        {
            // Inputs
        },
        {
            // Outputs
            std::make_tuple(OutputIndex::Measurement, std::vector<int>{dofs}, DataType::DOUBLE),
        },
    });

    if (!ok) {
        wbtError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool GetMeasurement::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!GetMeasurement::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    // Read the measured type
    std::string measuredType;
    if (!m_parameters.getParameter("MeasuredType", measuredType)) {
        wbtError << "Could not read measured type parameter.";
        return false;
    }

    // Set the measured type
    if (measuredType == "Joints Position") {
        pImpl->measuredType = impl::MeasuredType::JOINT_POS;
    }
    else if (measuredType == "Joints Velocity") {
        pImpl->measuredType = impl::MeasuredType::JOINT_VEL;
    }
    else if (measuredType == "Joints Acceleration") {
        pImpl->measuredType = impl::MeasuredType::JOINT_ACC;
    }
    else if (measuredType == "Joints Torque") {
        pImpl->measuredType = impl::MeasuredType::JOINT_TORQUE;
    }
    else if (measuredType == "Motor Position") {
        pImpl->measuredType = impl::MeasuredType::MOTOR_POS;
    }
    else if (measuredType == "Motor Velocity") {
        pImpl->measuredType = impl::MeasuredType::MOTOR_VEL;
    }
    else if (measuredType == "Motor Acceleration") {
        pImpl->measuredType = impl::MeasuredType::MOTOR_ACC;
    }
    else if (measuredType == "Motor Current") {
        pImpl->measuredType = impl::MeasuredType::MOTOR_CURRENT;
    }
    else if (measuredType == "Motor PWM") {
        pImpl->measuredType = impl::MeasuredType::MOTOR_PWM;
    }
    else {
        wbtError << "Measurement not supported.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    // Get the DoFs
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }
    const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();

    // Initialize the size of the output vector
    pImpl->measurement.resize(dofs);

    // Retain the ControlBoardRemapper
    if (!robotInterface->retainRemoteControlBoardRemapper()) {
        wbtError << "Couldn't retain the RemoteControlBoardRemapper.";
        return false;
    }

    return true;
}

bool GetMeasurement::terminate(const BlockInformation* blockInfo)
{
    // Get the RobotInterface
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }

    // Release the RemoteControlBoardRemapper
    bool ok = robotInterface->releaseRemoteControlBoardRemapper();
    if (!ok) {
        wbtError << "Failed to release the RemoteControlBoardRemapper.";
        // Don't return false here. WBBlock::terminate must be called in any case
    }

    return ok && WBBlock::terminate(blockInfo);
}

bool GetMeasurement::output(const BlockInformation* blockInfo)
{
    // Get the RobotInterface
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }

    bool ok;
    switch (pImpl->measuredType) {
            //
            // JOINT MEASUREMENTS
            // ==================
            //
        case impl::MeasuredType::JOINT_POS: {
            // Get the interface
            yarp::dev::IEncoders* iEncoders = nullptr;
            if (!robotInterface->getInterface(iEncoders) || !iEncoders) {
                wbtError << "Failed to get IPidControl interface.";
                return false;
            }
            // Get the measurement
            ok = iEncoders->getEncoders(pImpl->measurement.data());
            GetMeasurement::impl::deg2rad(pImpl->measurement);
            break;
        }
        case impl::MeasuredType::JOINT_VEL: {
            // Get the interface
            yarp::dev::IEncoders* iEncoders = nullptr;
            if (!robotInterface->getInterface(iEncoders) || !iEncoders) {
                wbtError << "Failed to get IEncoders interface.";
                return false;
            }
            // Get the measurement
            ok = iEncoders->getEncoderSpeeds(pImpl->measurement.data());
            GetMeasurement::impl::deg2rad(pImpl->measurement);
            break;
        }
        case impl::MeasuredType::JOINT_ACC: {
            // Get the interface
            yarp::dev::IEncoders* iEncoders = nullptr;
            if (!robotInterface->getInterface(iEncoders) || !iEncoders) {
                wbtError << "Failed to get IEncoders interface.";
                return false;
            }
            // Get the measurement
            ok = iEncoders->getEncoderAccelerations(pImpl->measurement.data());
            GetMeasurement::impl::deg2rad(pImpl->measurement);
            break;
        }
        case impl::MeasuredType::JOINT_TORQUE: {
            // Get the interface
            yarp::dev::ITorqueControl* iTorqueControl = nullptr;
            if (!robotInterface->getInterface(iTorqueControl) || !iTorqueControl) {
                wbtError << "Failed to get ITorqueControl interface.";
                return false;
            }
            // Get the measurement
            ok = iTorqueControl->getTorques(pImpl->measurement.data());
            break;
        }
        //
        // MOTOR MEASUREMENTS
        // ==================
        //
        case impl::MeasuredType::MOTOR_POS: {
            // Get the interface
            yarp::dev::IMotorEncoders* iMotorEncoders = nullptr;
            if (!robotInterface->getInterface(iMotorEncoders) || !iMotorEncoders) {
                wbtError << "Failed to get ITorqueControl interface.";
                return false;
            }
            // Get the measurement
            ok = iMotorEncoders->getMotorEncoders(pImpl->measurement.data());
            GetMeasurement::impl::deg2rad(pImpl->measurement);
            break;
        }
        case impl::MeasuredType::MOTOR_VEL: {
            // Get the interface
            yarp::dev::IMotorEncoders* iMotorEncoders = nullptr;
            if (!robotInterface->getInterface(iMotorEncoders) || !iMotorEncoders) {
                wbtError << "Failed to get ITorqueControl interface.";
                return false;
            }
            // Get the measurement
            ok = iMotorEncoders->getMotorEncoderSpeeds(pImpl->measurement.data());
            GetMeasurement::impl::deg2rad(pImpl->measurement);
            break;
        }
        case impl::MeasuredType::MOTOR_ACC: {
            // Get the interface
            yarp::dev::IMotorEncoders* iMotorEncoders = nullptr;
            if (!robotInterface->getInterface(iMotorEncoders) || !iMotorEncoders) {
                wbtError << "Failed to get ITorqueControl interface.";
                return false;
            }
            // Get the measurement
            ok = iMotorEncoders->getMotorEncoderAccelerations(pImpl->measurement.data());
            GetMeasurement::impl::deg2rad(pImpl->measurement);
            break;
        }
        case impl::MeasuredType::MOTOR_CURRENT: {
            // Get the interface
            yarp::dev::ICurrentControl* iCurrentControl = nullptr;
            if (!robotInterface->getInterface(iCurrentControl) || !iCurrentControl) {
                wbtError << "Failed to get ITorqueControl interface.";
                return false;
            }
            // Get the measurement
            ok = iCurrentControl->getCurrents(pImpl->measurement.data());
            break;
        }
        case impl::MeasuredType::MOTOR_PWM: {
            // Get the interface
            yarp::dev::IPWMControl* iPWMControl = nullptr;
            if (!robotInterface->getInterface(iPWMControl) || !iPWMControl) {
                wbtError << "Failed to get ITorqueControl interface.";
                return false;
            }
            // Get the measurement
            ok = iPWMControl->getDutyCycles(pImpl->measurement.data());
            break;
        }
    }

    if (!ok) {
        wbtError << "Failed to get measurement.";
        return false;
    }

    // Get the output signal
    Signal output = blockInfo->getOutputPortSignal(OutputIndex::Measurement);
    if (!output.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }

    // Fill the output buffer
    if (!output.setBuffer(pImpl->measurement.data(), output.getWidth())) {
        wbtError << "Failed to set output buffer.";
        return false;
    }

    return true;
}
