/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/ForwardKinematics.h"
#include "WBToolbox/Base/Configuration.h"
#include "WBToolbox/Base/RobotInterface.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <BlockFactory/Core/Signal.h>
#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Indices.h>

#include <ostream>
#include <tuple>

using namespace wbt::block;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = wbt::base::WBBlock::NumberOfParameters - 1,
    Frame
};

enum InputIndex
{
    BasePose = 0,
    JointConfiguration,
};

enum OutputIndex
{
    Transform = 0,
};

// BLOCK PIMPL
// ===========

class ForwardKinematics::impl
{
public:
    bool frameIsCoM = false;
    iDynTree::FrameIndex frameIndex = iDynTree::FRAME_INVALID_INDEX;
};

// BLOCK CLASS
// ===========

ForwardKinematics::ForwardKinematics()
    : pImpl{new impl()}
{}

ForwardKinematics::~ForwardKinematics() = default;

unsigned ForwardKinematics::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool ForwardKinematics::parseParameters(BlockInformation* blockInfo)
{
    const ParameterMetadata frameMetadata(ParameterType::STRING, ParamIndex::Frame, 1, 1, "Frame");

    if (!blockInfo->addParameterMetadata(frameMetadata)) {
        bfError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool ForwardKinematics::configureSizeAndPorts(BlockInformation* blockInfo)
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
    //
    // OUTPUTS
    // =======
    //
    // 1) Homogeneous transformation between the world and the specified frame (4x4 matrix)
    //

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
            {InputIndex::BasePose, Port::Dimensions{4, 4}, Port::DataType::DOUBLE},
            {InputIndex::JointConfiguration, Port::Dimensions{dofs}, Port::DataType::DOUBLE},
        },
        {
            // Outputs
            {OutputIndex::Transform, Port::Dimensions{4, 4}, Port::DataType::DOUBLE},
        });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool ForwardKinematics::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!ForwardKinematics::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    std::string frame;
    if (!m_parameters.getParameter("Frame", frame)) {
        bfError << "Cannot retrieve string from frame parameter.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    // Check if the frame is valid
    // ---------------------------

    auto kinDyn = getKinDynComputations();
    if (!kinDyn) {
        bfError << "Cannot retrieve handle to KinDynComputations.";
        return false;
    }

    if (frame != "com") {
        pImpl->frameIndex = kinDyn->getFrameIndex(frame);
        if (pImpl->frameIndex == iDynTree::FRAME_INVALID_INDEX) {
            bfError << "Cannot find " + frame + " in the frame list.";
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
    auto kinDyn = getKinDynComputations();
    if (!kinDyn) {
        bfError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    InputSignalPtr basePoseSig = blockInfo->getInputPortSignal(InputIndex::BasePose);
    InputSignalPtr jointsPosSig = blockInfo->getInputPortSignal(InputIndex::JointConfiguration);

    if (!basePoseSig || !jointsPosSig) {
        bfError << "Input signals not valid.";
        return false;
    }

    bool ok = setRobotState(basePoseSig, jointsPosSig, nullptr, nullptr, kinDyn.get());

    if (!ok) {
        bfError << "Failed to set the robot state.";
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
    OutputSignalPtr output = blockInfo->getOutputPortSignal(OutputIndex::Transform);
    if (!output) {
        bfError << "Output signal not valid.";
        return false;
    }

    // Allocate objects for row-major -> col-major conversion
    Map<const Matrix4diDynTree> world_H_frame_RowMajor =
        toEigen(world_H_frame.asHomogeneousTransform());
    Map<Matrix4dSimulink> world_H_frame_ColMajor(output->getBuffer<double>(), 4, 4);

    // Forward the buffer to Simulink transforming it to ColMajor
    world_H_frame_ColMajor = world_H_frame_RowMajor;
    return true;
}
