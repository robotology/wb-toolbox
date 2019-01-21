/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/InverseDynamics.h"
#include "WBToolbox/Base/Configuration.h"
#include "WBToolbox/Base/RobotInterface.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Signal.h>
#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/Model/JointState.h>
#include <iDynTree/Model/LinkState.h>

#include <memory>
#include <ostream>
#include <tuple>

using namespace wbt::block;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum InputIndex
{
    BasePose = 0,
    JointConfiguration,
    BaseVelocity,
    JointVelocity,
    BaseAcceleration,
    JointAcceleration,
};

enum OutputIndex
{
    Torques = 0,
};

// BLOCK PIMPL
// ===========

class InverseDynamics::impl
{
public:
    iDynTree::Vector6 baseAcceleration;
    iDynTree::VectorDynSize jointsAcceleration;
    iDynTree::FreeFloatingGeneralizedTorques torques;
};

// BLOCK CLASS
// ===========

InverseDynamics::InverseDynamics()
    : pImpl{new impl()}
{}

InverseDynamics::~InverseDynamics() = default;

unsigned InverseDynamics::numberOfParameters()
{
    return WBBlock::numberOfParameters();
}

bool InverseDynamics::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // Get the DoFs
    const int dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    // INPUTS
    // ======
    //
    // 1) Homogeneous transform for base pose wrt the world frame (4x4 matrix)
    // 2) Joints position (1xDoFs vector)
    // 3) Base frame velocity (1x6 vector)
    // 4) Joints velocity (1xDoFs vector)
    // 5) Base frame acceleration (1x6 vector)
    // 6) Joints acceleration (1xDoFs vector)
    //
    // OUTPUTS
    // =======
    //
    // 1) Vector representing the torques (1x(DoFs+6))
    //

    const bool ok = blockInfo->setIOPortsData({
        {
            // Inputs
            std::make_tuple(InputIndex::BasePose, std::vector<int>{4, 4}, DataType::DOUBLE),
            std::make_tuple(
                InputIndex::JointConfiguration, std::vector<int>{dofs}, DataType::DOUBLE),
            std::make_tuple(InputIndex::BaseVelocity, std::vector<int>{6}, DataType::DOUBLE),
            std::make_tuple(InputIndex::JointVelocity, std::vector<int>{dofs}, DataType::DOUBLE),
            std::make_tuple(InputIndex::BaseAcceleration, std::vector<int>{6}, DataType::DOUBLE),
            std::make_tuple(
                InputIndex::JointAcceleration, std::vector<int>{dofs}, DataType::DOUBLE),
        },
        {
            // Outputs
            std::make_tuple(OutputIndex::Torques, std::vector<int>{dofs + 6}, DataType::DOUBLE),
        },
    });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool InverseDynamics::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    using namespace iDynTree;

    // Get the DoFs
    const auto dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    // Initialize sizes and value
    pImpl->baseAcceleration.zero();
    pImpl->jointsAcceleration.resize(dofs);
    pImpl->jointsAcceleration.zero();

    // Get the KinDynComputations pointer
    const auto& kindyn = getRobotInterface()->getKinDynComputations();
    if (!kindyn) {
        bfError << "Failed to get the KinDynComputations object";
        return false;
    }

    // Get the model from the KinDynComputations object
    const auto& model = kindyn->model();

    // Initialize the output object
    pImpl->torques = FreeFloatingGeneralizedTorques(model);

    return true;
}

bool InverseDynamics::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool InverseDynamics::output(const BlockInformation* blockInfo)
{
    // Get the KinDynComputations object
    auto kinDyn = getKinDynComputations();
    if (!kinDyn) {
        bfError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    InputSignalPtr basePoseSig = blockInfo->getInputPortSignal(InputIndex::BasePose);
    InputSignalPtr jointsPosSig = blockInfo->getInputPortSignal(InputIndex::JointConfiguration);
    InputSignalPtr baseVelocitySignal = blockInfo->getInputPortSignal(InputIndex::BaseVelocity);
    InputSignalPtr jointsVelocitySignal = blockInfo->getInputPortSignal(InputIndex::JointVelocity);

    if (!basePoseSig || !jointsPosSig || !baseVelocitySignal || !jointsVelocitySignal) {
        bfError << "Input signals not valid.";
        return false;
    }

    bool ok = setRobotState(
        basePoseSig, jointsPosSig, baseVelocitySignal, jointsVelocitySignal, kinDyn.get());

    if (!ok) {
        bfError << "Failed to set the robot state.";
        return false;
    }

    // Base acceleration
    // -----------------

    InputSignalPtr baseAccelerationSignal =
        blockInfo->getInputPortSignal(InputIndex::BaseAcceleration);
    if (!baseAccelerationSignal) {
        bfError << "Base Acceleration signal not valid.";
        return false;
    }
    const double* bufBaseAcc = baseAccelerationSignal->getBuffer<double>();

    for (unsigned i = 0; i < baseAccelerationSignal->getWidth(); ++i) {
        if (!pImpl->baseAcceleration.setVal(i, bufBaseAcc[i])) {
            bfError << "Failed to fill base accelerations class member.";
            return false;
        }
    }

    // Joints acceleration
    // -------------------

    InputSignalPtr jointsAccelerationSignal =
        blockInfo->getInputPortSignal(InputIndex::JointAcceleration);
    if (!jointsAccelerationSignal) {
        bfError << "Joints Acceleration signal not valid.";
        return false;
    }
    const double* bufJointsAcc = jointsAccelerationSignal->getBuffer<double>();

    for (unsigned i = 0; i < jointsAccelerationSignal->getWidth(); ++i) {
        if (!pImpl->jointsAcceleration.setVal(i, bufJointsAcc[i])) {
            bfError << "Failed to fill joint accelerations class member.";
            return false;
        }
    }

    // OUTPUT
    // ======

    // Calculate the inverse dynamics (assuming zero external forces)
    ok = kinDyn->inverseDynamics(pImpl->baseAcceleration,
                                 pImpl->jointsAcceleration,
                                 iDynTree::LinkNetExternalWrenches(kinDyn->getNrOfLinks()),
                                 pImpl->torques);

    if (!ok) {
        bfError << "iDynTree failed to compute inverse dynamics.";
        return false;
    }

    // Get the output signal
    OutputSignalPtr output = blockInfo->getOutputPortSignal(OutputIndex::Torques);
    if (!output) {
        bfError << "Output signal not valid.";
        return false;
    }
    double* outputBuffer = output->getBuffer<double>();

    // Convert generalized torques and forward the directly to Simulink
    // mapping the memory through Eigen::Map
    const auto& torquesSize = pImpl->torques.jointTorques().size();
    Eigen::Map<Eigen::VectorXd> generalizedOutputTrqs(outputBuffer, torquesSize + 6);
    generalizedOutputTrqs.segment(0, 6) = toEigen(pImpl->torques.baseWrench());
    generalizedOutputTrqs.segment(6, torquesSize) = toEigen(pImpl->torques.jointTorques());

    return true;
}
