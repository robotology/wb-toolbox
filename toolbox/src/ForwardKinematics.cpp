/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "ForwardKinematics.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "Parameter.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Indices.h>

#include <memory>

using namespace wbt;

const std::string ForwardKinematics::ClassName = "ForwardKinematics";

const unsigned INPUT_IDX_BASE_POSE = 0;
const unsigned INPUT_IDX_JOINTCONF = 1;
const unsigned OUTPUT_IDX_FW_FRAME = 0;

const unsigned PARAM_IDX_BIAS = WBBlock::NumberOfParameters - 1;
const unsigned PARAM_IDX_FRAME = PARAM_IDX_BIAS + 1;

class ForwardKinematics::impl
{
public:
    bool frameIsCoM = false;
    iDynTree::FrameIndex frameIndex = iDynTree::FRAME_INVALID_INDEX;
};

ForwardKinematics::ForwardKinematics()
    : pImpl{new impl()}
{}

unsigned ForwardKinematics::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool ForwardKinematics::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata frameMetadata(ParameterType::STRING, PARAM_IDX_FRAME, 1, 1, "Frame");

    if (!blockInfo->addParameterMetadata(frameMetadata)) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool ForwardKinematics::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // Memory allocation / Saving data not allowed here

    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // INPUTS
    // ======
    //
    // 1) Homogeneous transform for base pose wrt the world frame (4x4 matrix)
    // 2) Joints position (1xDoFs vector)
    //

    // Number of inputs
    if (!blockInfo->setNumberOfInputPorts(2)) {
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
    bool ok = true;
    ok = ok && blockInfo->setInputPortMatrixSize(INPUT_IDX_BASE_POSE, {4, 4});
    ok = ok && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINTCONF, dofs);

    blockInfo->setInputPortType(INPUT_IDX_BASE_POSE, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINTCONF, DataType::DOUBLE);

    if (!ok) {
        wbtError << "Failed to configure input ports.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Homogeneous transformation between the world and the specified frame (4x4 matrix)
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        wbtError << "Failed to configure the number of output ports.";
        return false;
    }

    // Size and type
    ok = blockInfo->setOutputPortMatrixSize(OUTPUT_IDX_FW_FRAME, {4, 4});
    blockInfo->setOutputPortType(OUTPUT_IDX_FW_FRAME, DataType::DOUBLE);

    return ok;
}

bool ForwardKinematics::initialize(BlockInformation* blockInfo)
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

    std::string frame;
    if (!m_parameters.getParameter("Frame", frame)) {
        wbtError << "Cannot retrieve string from frame parameter.";
        return false;
    }

    // Check if the frame is valid
    // ---------------------------

    auto kinDyn = getKinDynComputations(blockInfo).lock();
    if (!kinDyn) {
        wbtError << "Cannot retrieve handle to KinDynComputations.";
        return false;
    }

    if (frame != "com") {
        pImpl->frameIndex = kinDyn->getFrameIndex(frame);
        if (pImpl->frameIndex == iDynTree::FRAME_INVALID_INDEX) {
            wbtError << "Cannot find " + frame + " in the frame list.";
            return false;
        }
    }
    else {
        pImpl->frameIsCoM = true;
        pImpl->frameIndex = iDynTree::FRAME_INVALID_INDEX;
    }

    return true;
}

bool ForwardKinematics::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool ForwardKinematics::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    using Matrix4dSimulink = Matrix<double, 4, 4, Eigen::ColMajor>;
    using Matrix4diDynTree = Matrix<double, 4, 4, Eigen::RowMajor>;

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

    if (!basePoseSig.isValid() || !jointsPosSig.isValid()) {
        wbtError << "Input signals not valid.";
        return false;
    }

    bool ok = setRobotState(&basePoseSig, &jointsPosSig, nullptr, nullptr, kinDyn.get());

    if (!ok) {
        wbtError << "Failed to set the robot state.";
        return false;
    }

    // OUTPUT
    // ======

    iDynTree::Transform world_H_frame;

    // Compute the transform to the selected frame
    if (!pImpl->frameIsCoM) {
        world_H_frame = kinDyn->getWorldTransform(pImpl->frameIndex);
    }
    else {
        world_H_frame.setPosition(kinDyn->getCenterOfMassPosition());
    }

    // Get the output signal memory location
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_FW_FRAME);
    if (!output.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }

    // Allocate objects for row-major -> col-major conversion
    Map<const Matrix4diDynTree> world_H_frame_RowMajor =
        toEigen(world_H_frame.asHomogeneousTransform());
    Map<Matrix4dSimulink> world_H_frame_ColMajor(output.getBuffer<double>(), 4, 4);

    // Forward the buffer to Simulink transforming it to ColMajor
    world_H_frame_ColMajor = world_H_frame_RowMajor;
    return true;
}
