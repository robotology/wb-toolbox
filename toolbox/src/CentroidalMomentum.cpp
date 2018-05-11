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
#include <tuple>

using namespace wbt;
const std::string CentroidalMomentum::ClassName = "CentroidalMomentum";

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

bool CentroidalMomentum::configureSizeAndPorts(BlockInformation* blockInfo)
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

    const bool ok = blockInfo->setIOPortsData({
        {
            // Inputs
            std::make_tuple(InputIndex::BasePose, std::vector<int>{4, 4}, DataType::DOUBLE),
            std::make_tuple(
                InputIndex::JointConfiguration, std::vector<int>{dofs}, DataType::DOUBLE),
            std::make_tuple(InputIndex::BaseVelocity, std::vector<int>{6}, DataType::DOUBLE),
            std::make_tuple(InputIndex::JointVelocity, std::vector<int>{dofs}, DataType::DOUBLE),
        },
        {
            // Outputs
            std::make_tuple(OutputIndex::CentroidalMomentum, std::vector<int>{6}, DataType::DOUBLE),
        },
    });

    if (!ok) {
        wbtError << "Failed to configure input / output ports.";
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
    auto kinDyn = getKinDynComputations(blockInfo).lock();
    if (!kinDyn) {
        wbtError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    const Signal basePoseSig = blockInfo->getInputPortSignal(InputIndex::BasePose);
    const Signal jointsPosSig = blockInfo->getInputPortSignal(InputIndex::JointConfiguration);
    const Signal baseVelocitySignal = blockInfo->getInputPortSignal(InputIndex::BaseVelocity);
    const Signal jointsVelocitySignal = blockInfo->getInputPortSignal(InputIndex::JointVelocity);

    if (!basePoseSig.isValid() || !jointsPosSig.isValid() || !baseVelocitySignal.isValid()
        || !jointsVelocitySignal.isValid()) {
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
    Signal output = blockInfo->getOutputPortSignal(OutputIndex::CentroidalMomentum);
    if (!output.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }

    // Fill the output buffer
    if (!output.setBuffer(toEigen(pImpl->centroidalMomentum).data(), output.getWidth())) {
        wbtError << "Failed to set output buffer.";
        return false;
    }

    return true;
}
