/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/GetLimits.h"
#include "WBToolbox/Base/Configuration.h"
#include "WBToolbox/Base/RobotInterface.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <BlockFactory/Core/Signal.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/IJoint.h>
#include <iDynTree/Indices.h>
#include <iDynTree/Model.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <cmath>
#include <limits>
#include <ostream>
#include <tuple>
#include <vector>

using namespace wbt::block;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = wbt::base::WBBlock::NumberOfParameters - 1,
    LimitType
};

enum OutputIndex
{
    MinLimit = 0,
    MaxLimit,
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

    bool firstRun;

    static double deg2rad(const double& v) { return v * M_PI / 180.0; }
};

// BLOCK CLASS
// ===========

GetLimits::GetLimits()
    : pImpl{new impl()}
{}

GetLimits::~GetLimits() = default;

unsigned GetLimits::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool GetLimits::parseParameters(BlockInformation* blockInfo)
{
    const ParameterMetadata limitTypeMetadata(
        ParameterType::STRING, ParamIndex::LimitType, 1, 1, "LimitType");

    if (!blockInfo->addParameterMetadata(limitTypeMetadata)) {
        bfError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool GetLimits::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // Get the DoFs
    const int dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    // INPUTS
    // ======
    //
    // No inputs
    //
    // OUTPUTS
    // =======
    //
    // 1) Vector with the min limit (1xDoFs)
    // 2) Vector with the max limit (1xDoFs)
    //

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
        },
        {
            // Outputs
            {OutputIndex::MinLimit, Port::Dimensions{dofs}, Port::DataType::DOUBLE},
            {OutputIndex::MaxLimit, Port::Dimensions{dofs}, Port::DataType::DOUBLE},
        });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
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
        bfError << "Failed to parse parameters.";
        return false;
    }

    // Read the control type
    if (!m_parameters.getParameter("LimitType", pImpl->limitType)) {
        bfError << "Failed to get parameters after their parsing.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    // Get the DoFs
    const auto dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    // Initialize the structure that stores the limits
    pImpl->limits.min.resize(dofs);
    pImpl->limits.max.resize(dofs);

    return true;
}

bool GetLimits::initializeInitialConditions(const BlockInformation* /*blockInfo*/)
{
    pImpl->firstRun = true;
    return true;
}

bool GetLimits::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool GetLimits::output(const BlockInformation* blockInfo)
{
    // Read the limits only during the first run
    if (pImpl->firstRun) {
        pImpl->firstRun = false;

        // Initializes some buffers
        double min = 0;
        double max = 0;

        // Get the RobotInterface and the DoFs
        const auto robotInterface = getRobotInterface();
        const int dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

        // From the RemoteControlBoardRemapper
        // ===================================
        //
        // In the next methods, the values are asked using joint index and not string.
        // The ControlBoardRemapper internally uses the same joints ordering of its
        // initialization. In this case, it matches 1:1 the controlled joint vector.
        // It is hence possible using i to point to the correct joint.

        // Get the IControlLimits2 interface
        yarp::dev::IControlLimits* iControlLimits2 = nullptr;
        if (pImpl->limitType == "ControlBoardPosition"
            || pImpl->limitType == "ControlBoardVelocity") {
            // Get the interface
            if (!robotInterface->getInterface(iControlLimits2) || !iControlLimits2) {
                bfError << "Failed to get IControlLimits2 interface.";
                return false;
            }
        }

        if (pImpl->limitType == "ControlBoardPosition") {
            for (unsigned i = 0; i < dofs; ++i) {
                if (!iControlLimits2->getLimits(i, &min, &max)) {
                    bfError << "Failed to get limits from the interface.";
                    return false;
                }
                pImpl->limits.min[i] = GetLimits::impl::deg2rad(min);
                pImpl->limits.max[i] = GetLimits::impl::deg2rad(max);
            }
        }
        else if (pImpl->limitType == "ControlBoardVelocity") {
            for (unsigned i = 0; i < dofs; ++i) {
                if (!iControlLimits2->getVelLimits(i, &min, &max)) {
                    bfError << "Failed to get limits from the interface.";
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
                bfError << "Failed to retrieve the KinDynComputations object.";
                return false;
            }

            // Get the model
            const iDynTree::Model model = kindyncomp->model();

            for (unsigned i = 0; i < dofs; ++i) {
                // Get the joint name
                const std::string joint =
                    robotInterface->getConfiguration().getControlledJoints()[i];

                // Get its index
                iDynTree::JointIndex jointIndex = model.getJointIndex(joint);

                if (jointIndex == iDynTree::JOINT_INVALID_INDEX) {
                    bfError << "Invalid iDynTree joint index.";
                    return false;
                }

                // Get the joint from the model
                p_joint = model.getJoint(jointIndex);

                if (!p_joint->hasPosLimits()) {
                    bfWarning << "Joint " << joint << " has no model limits.";
                    pImpl->limits.min[i] = -std::numeric_limits<double>::infinity();
                    pImpl->limits.max[i] = std::numeric_limits<double>::infinity();
                }
                else {
                    if (!p_joint->getPosLimits(0, min, max)) {
                        bfError << "Failed to get joint limits from the URDF model "
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
            bfError << "Limit type " + pImpl->limitType + " not recognized.";
            return false;
        }
    }

    OutputSignalPtr minPort = blockInfo->getOutputPortSignal(OutputIndex::MinLimit);
    OutputSignalPtr maxPort = blockInfo->getOutputPortSignal(OutputIndex::MaxLimit);

    if (!minPort || !maxPort) {
        bfError << "Output signals not valid.";
        return false;
    }

    // Get the DoFs
    const auto dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    bool ok = true;
    ok = ok && minPort->setBuffer(pImpl->limits.min.data(), dofs);
    ok = ok && maxPort->setBuffer(pImpl->limits.max.data(), dofs);

    if (!ok) {
        bfError << "Failed to set output buffers.";
        return false;
    }

    return true;
}
