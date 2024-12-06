/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/CentroidalMomentum.h"
#include "WBToolbox/Base/Configuration.h"
#include "WBToolbox/Base/RobotInterface.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Signal.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/SpatialMomentum.h>
#include <iDynTree/KinDynComputations.h>

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
};

enum OutputIndex
{
    CentroidalMomentum = 0,
};

// BLOCK PIMPL
// ===========

class CentroidalMomentum::impl
{
public:
    iDynTree::SpatialMomentum centroidalMomentum;
};

// BLOCK CLASS
// ===========

CentroidalMomentum::CentroidalMomentum()
    : pImpl{new impl()}
{}

CentroidalMomentum::~CentroidalMomentum() = default;

bool CentroidalMomentum::configureSizeAndPorts(BlockInformation* blockInfo)
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
    //
    // OUTPUTS
    // =======
    //
    // 1) Vector representing the centroidal momentum (1x6)
    //

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
            {InputIndex::BasePose, Port::Dimensions{4, 4}, Port::Port::DataType::DOUBLE},
            {InputIndex::JointConfiguration, Port::Dimensions{dofs}, Port::Port::DataType::DOUBLE},
            {InputIndex::BaseVelocity, Port::Dimensions{6}, Port::Port::DataType::DOUBLE},
            {InputIndex::JointVelocity, Port::Dimensions{dofs}, Port::Port::DataType::DOUBLE},
        },
        {
            // Outputs
            {OutputIndex::CentroidalMomentum, Port::Dimensions{6}, Port::Port::DataType::DOUBLE},
        });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool CentroidalMomentum::initialize(BlockInformation* blockInfo)
{
    return WBBlock::initialize(blockInfo);
}

bool CentroidalMomentum::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool CentroidalMomentum::output(const BlockInformation* blockInfo)
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

    // OUTPUT
    // ======

    // Calculate the centroidal momentum
    pImpl->centroidalMomentum = kinDyn->getCentroidalTotalMomentum();

    // Get the output signal
    OutputSignalPtr output = blockInfo->getOutputPortSignal(OutputIndex::CentroidalMomentum);
    if (!output) {
        bfError << "Output signal not valid.";
        return false;
    }

    // Fill the output buffer
    if (!output->setBuffer(toEigen(pImpl->centroidalMomentum).data(), output->getWidth())) {
        bfError << "Failed to set output buffer.";
        return false;
    }

    return true;
}
