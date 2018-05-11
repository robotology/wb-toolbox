/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "GetLimits.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/IJoint.h>
#include <iDynTree/Model/Indices.h>
#include <iDynTree/Model/Model.h>
#include <yarp/dev/IControlLimits2.h>

#include <cmath>
#include <limits>
#include <ostream>
#include <vector>

using namespace wbt;
const std::string GetLimits::ClassName = "GetLimits";

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = WBBlock::NumberOfParameters - 1,
    LimitType
};

// BLOCK PIMPL
// ===========

class GetLimits::impl
{
private:
    struct Limit
    {
        std::vector<double> min;
        std::vector<double> max;
    };

public:
    Limit limits;
    std::string limitType;

    static double deg2rad(const double& v) { return v * M_PI / 180.0; }
};

// BLOCK CLASS
// ===========

wbt::GetLimits::GetLimits()
    : pImpl{new impl()}
{}

unsigned GetLimits::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool GetLimits::parseParameters(BlockInformation* blockInfo)
{
    const ParameterMetadata limitTypeMetadata(
        ParameterType::STRING, ParamIndex::LimitType, 1, 1, "LimitType");

    if (!blockInfo->addParameterMetadata(limitTypeMetadata)) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool GetLimits::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // INPUTS
    // ======
    //
    // No inputs
    //

    if (!blockInfo->setNumberOfInputPorts(0)) {
        wbtError << "Failed to configure the number of input ports.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) vector with the information asked (1xDoFs)
    //

    if (!blockInfo->setNumberOfOutputPorts(2)) {
        wbtError << "Failed to configure the number of output ports.";
        return false;
    }

    // Get the DoFs
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }
    const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();

    bool success = true;
    success = success && blockInfo->setOutputPortVectorSize(0, dofs); // Min limit
    success = success && blockInfo->setOutputPortVectorSize(1, dofs); // Max limit

    blockInfo->setOutputPortType(0, DataType::DOUBLE);
    blockInfo->setOutputPortType(1, DataType::DOUBLE);

    if (!success) {
        wbtError << "Failed to configure output ports.";
        return false;
    }

    return true;
}

bool GetLimits::initialize(BlockInformation* blockInfo)
{
    using namespace yarp::os;

    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!GetLimits::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    // Read the control type
    if (!m_parameters.getParameter("LimitType", pImpl->limitType)) {
        wbtError << "Failed to get parameters after their parsing.";
        return false;
    }

    // Get the DoFs
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }
    const auto& configuration = robotInterface->getConfiguration();
    const auto dofs = configuration.getNumberOfDoFs();

    // Initialize the structure that stores the limits
    pImpl->limits.min.resize(dofs);
    pImpl->limits.max.resize(dofs);

    // Initializes some buffers
    double min = 0;
    double max = 0;

    // From the RemoteControlBoardRemapper
    // ===================================
    //
    // In the next methods, the values are asked using joint index and not string.
    // The ControlBoardRemapper internally uses the same joints ordering of its
    // initialization. In this case, it matches 1:1 the controlled joint vector.
    // It is hence possible using i to point to the correct joint.

    // Get the RemoteControlBoardRemapper and IControlLimits2 interface if needed
    yarp::dev::IControlLimits2* iControlLimits2 = nullptr;
    if (pImpl->limitType == "ControlBoardPosition" || pImpl->limitType == "ControlBoardVelocity") {
        // Retain the control board remapper
        if (!robotInterface->retainRemoteControlBoardRemapper()) {
            wbtError << "Couldn't retain the RemoteControlBoardRemapper.";
            return false;
        }
        // Get the interface
        if (!robotInterface->getInterface(iControlLimits2) || !iControlLimits2) {
            wbtError << "Failed to get IControlLimits2 interface.";
            return false;
        }
    }

    if (pImpl->limitType == "ControlBoardPosition") {
        for (auto i = 0; i < dofs; ++i) {
            if (!iControlLimits2->getLimits(i, &min, &max)) {
                wbtError << "Failed to get limits from the interface.";
                return false;
            }
            pImpl->limits.min[i] = GetLimits::impl::deg2rad(min);
            pImpl->limits.max[i] = GetLimits::impl::deg2rad(max);
        }
    }
    else if (pImpl->limitType == "ControlBoardVelocity") {
        for (auto i = 0; i < dofs; ++i) {
            if (!iControlLimits2->getVelLimits(i, &min, &max)) {
                wbtError << "Failed to get limits from the interface.";
                return false;
            }
            pImpl->limits.min[i] = GetLimits::impl::deg2rad(min);
            pImpl->limits.max[i] = GetLimits::impl::deg2rad(max);
        }
    }

    // From the URDF model
    // ===================
    //
    // For the time being, only position limits are supported.

    else if (pImpl->limitType == "ModelPosition") {
        iDynTree::IJointConstPtr p_joint;

        // Get the KinDynComputations pointer
        const auto& kindyncomp = robotInterface->getKinDynComputations();
        if (!kindyncomp) {
            wbtError << "Failed to retrieve the KinDynComputations object.";
            return false;
        }

        // Get the model
        const iDynTree::Model model = kindyncomp->model();

        for (unsigned i = 0; i < dofs; ++i) {
            // Get the joint name
            const std::string joint = configuration.getControlledJoints()[i];

            // Get its index
            iDynTree::JointIndex jointIndex = model.getJointIndex(joint);

            if (jointIndex == iDynTree::JOINT_INVALID_INDEX) {
                wbtError << "Invalid iDynTree joint index.";
                return false;
            }

            // Get the joint from the model
            p_joint = model.getJoint(jointIndex);

            if (!p_joint->hasPosLimits()) {
                wbtWarning << "Joint " << joint << " has no model limits.";
                pImpl->limits.min[i] = -std::numeric_limits<double>::infinity();
                pImpl->limits.max[i] = std::numeric_limits<double>::infinity();
            }
            else {
                if (!p_joint->getPosLimits(0, min, max)) {
                    wbtError << "Failed to get joint limits from the URDF model "
                             << "for the joint " << joint + ".";
                    return false;
                }
                pImpl->limits.min[i] = min;
                pImpl->limits.max[i] = max;
            }
        }
    }
    // TODO: other limits from the model?
    // else if (limitType == "ModelVelocity") {
    // }
    // else if (limitType == "ModelEffort") {
    // }
    else {
        wbtError << "Limit type " + pImpl->limitType + " not recognized.";
        return false;
    }

    return true;
}

bool GetLimits::terminate(const BlockInformation* blockInfo)
{
    bool ok = true;

    // Release the RemoteControlBoardRemapper
    if (pImpl->limitType == "ControlBoardPosition" || pImpl->limitType == "ControlBoardVelocity") {
        auto robotInterface = getRobotInterface(blockInfo).lock();
        if (!robotInterface || !robotInterface->releaseRemoteControlBoardRemapper()) {
            wbtError << "Failed to release the RemoteControlBoardRemapper.";
            ok = false;
            // Don't return false here. WBBlock::terminate must be called in any case
        }
    }

    return ok && WBBlock::terminate(blockInfo);
}

bool GetLimits::output(const BlockInformation* blockInfo)
{
    Signal minPort = blockInfo->getOutputPortSignal(0);
    Signal maxPort = blockInfo->getOutputPortSignal(1);

    if (!minPort.isValid() || !maxPort.isValid()) {
        wbtError << "Output signals not valid.";
        return false;
    }

    // Get the Configuration
    auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }
    const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();

    bool ok = true;

    ok = ok && minPort.setBuffer(pImpl->limits.min.data(), dofs);
    ok = ok && maxPort.setBuffer(pImpl->limits.max.data(), dofs);

    if (!ok) {
        wbtError << "Failed to set output buffers.";
        return false;
    }

    return true;
}
