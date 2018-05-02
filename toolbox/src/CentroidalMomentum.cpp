/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "CentroidalMomentum.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/KinDynComputations.h>

#include <ostream>

using namespace wbt;

const std::string CentroidalMomentum::ClassName = "CentroidalMomentum";

const unsigned INPUT_IDX_BASE_POSE = 0;
const unsigned INPUT_IDX_JOINTCONF = 1;
const unsigned INPUT_IDX_BASE_VEL = 2;
const unsigned INPUT_IDX_JOINT_VEL = 3;
const unsigned OUTPUT_IDX_CENTRMOM = 0;

class CentroidalMomentum::impl
{
public:
    iDynTree::SpatialMomentum centroidalMomentum;
};

CentroidalMomentum::CentroidalMomentum()
    : pImpl{new impl()}
{}

bool CentroidalMomentum::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // INPUTS
    // ======
    //
    // 1) Homogeneous transform for base pose wrt the world frame (4x4 matrix)
    // 2) Joints position (1xDoFs vector)
    // 3) Base frame velocity (1x6 vector)
    // 4) Joints velocity (1xDoFs vector)
    //

    // Number of inputs
    if (!blockInfo->setNumberOfInputPorts(4)) {
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
    bool success = true;
    success = success && blockInfo->setInputPortMatrixSize(INPUT_IDX_BASE_POSE, {4, 4});
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINTCONF, dofs);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_BASE_VEL, 6);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINT_VEL, dofs);

    blockInfo->setInputPortType(INPUT_IDX_BASE_POSE, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINTCONF, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_BASE_VEL, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINT_VEL, DataType::DOUBLE);

    if (!success) {
        wbtError << "Failed to configure input ports.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Vector representing the centroidal momentum (1x6)
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        wbtError << "Failed to configure the number of output ports.";
        return false;
    }

    // Size and type
    success = blockInfo->setOutputPortVectorSize(OUTPUT_IDX_CENTRMOM, 6);
    blockInfo->setOutputPortType(OUTPUT_IDX_CENTRMOM, DataType::DOUBLE);

    return success;
}

bool CentroidalMomentum::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    return true;
}

bool CentroidalMomentum::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool CentroidalMomentum::output(const BlockInformation* blockInfo)
{
    // Get the KinDynComputations object
    auto kinDyn = getKinDynComputations(blockInfo).lock();
    if (!kinDyn) {
        wbtError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    const Signal basePoseSig = blockInfo->getInputPortSignal(INPUT_IDX_BASE_POSE);
    const Signal jointsPosSig = blockInfo->getInputPortSignal(INPUT_IDX_JOINTCONF);
    const Signal baseVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_BASE_VEL);
    const Signal jointsVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_JOINT_VEL);

    if (!basePoseSig.isValid() || !jointsPosSig.isValid() || baseVelocitySignal.isValid()
        || jointsVelocitySignal.isValid()) {
        wbtError << "Input signals not valid.";
        return false;
    }

    bool ok = setRobotState(
        &basePoseSig, &jointsPosSig, &baseVelocitySignal, &jointsVelocitySignal, kinDyn.get());

    if (!ok) {
        wbtError << "Failed to set the robot state.";
        return false;
    }

    // OUTPUT
    // ======

    // Calculate the centroidal momentum
    pImpl->centroidalMomentum = kinDyn->getCentroidalTotalMomentum();

    // Get the output signal
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_CENTRMOM);
    if (!output.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }

    // Fill the output buffer
    if (!output.setBuffer(toEigen(pImpl->centroidalMomentum).data(), output.getWidth())) {
        wbtError << "Failed to set output buffer.";
    }

    return true;
}
