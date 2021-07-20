/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/RelativeJacobian.h"
#include "WBToolbox/Base/Configuration.h"
#include "WBToolbox/Base/RobotInterface.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <BlockFactory/Core/Signal.h>
#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Indices.h>

#include <memory>
#include <ostream>
#include <tuple>

using namespace wbt::block;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = wbt::base::WBBlock::NumberOfParameters - 1,
    Frame1,
    Frame2
};

enum InputIndex
{
    BasePose = 0,
    JointConfiguration,
};

enum OutputIndex
{
    RelativeJacobian = 0,
};

// BLOCK PIMPL
// ===========

class RelativeJacobian::impl
{
public:
    iDynTree::MatrixDynSize jacobianCOM;
    iDynTree::MatrixDynSize jacobian;

    bool frameIsCoM = false;
    iDynTree::FrameIndex frameIndex1 = iDynTree::FRAME_INVALID_INDEX;
    iDynTree::FrameIndex frameIndex2 = iDynTree::FRAME_INVALID_INDEX;
};

// BLOCK CLASS
// ===========

RelativeJacobian::RelativeJacobian()
    : pImpl{new impl()}
{}

RelativeJacobian::~RelativeJacobian() = default;

unsigned RelativeJacobian::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 2;
}

bool RelativeJacobian::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> frameMetadata{
    {ParameterType::STRING, ParamIndex::Frame1, 1, 1, "Frame1"},
    {ParameterType::STRING, ParamIndex::Frame2, 1, 1, "Frame2"}};

    for (const auto& md : frameMetadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            bfError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool RelativeJacobian::configureSizeAndPorts(BlockInformation* blockInfo)
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
    //
    // OUTPUTS
    // =======
    //
    // 1) Matrix representing the Jacobian (6x(DoFs))
    //

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
            {InputIndex::BasePose, Port::Dimensions{4, 4}, Port::DataType::DOUBLE},

            {InputIndex::JointConfiguration, Port::Dimensions{dofs}, Port::DataType::DOUBLE},
        },
        {
            // Outputs
            {OutputIndex::RelativeJacobian, Port::Dimensions{6, dofs}, Port::DataType::DOUBLE},
        });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool RelativeJacobian::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!RelativeJacobian::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    std::string frame1, frame2;
    if (!m_parameters.getParameter("Frame1", frame1)) {
        bfError << "Cannot retrieve string from the first frame parameter.";
        return false;
    }

    if (!m_parameters.getParameter("Frame2", frame2)) {
        bfError << "Cannot retrieve string from the second frame parameter.";
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

    if (frame1 != "com") {
        pImpl->frameIndex1 = kinDyn->getFrameIndex(frame1);
        if (pImpl->frameIndex1 == iDynTree::FRAME_INVALID_INDEX) {
            bfError << "Cannot find " + frame1 + " in the frame list.";
            return false;
        }
    }
    else {
        pImpl->frameIsCoM = true;
        pImpl->frameIndex1 = iDynTree::FRAME_INVALID_INDEX;
    }

    if (frame2 != "com") {
        pImpl->frameIndex2 = kinDyn->getFrameIndex(frame2);
        if (pImpl->frameIndex2 == iDynTree::FRAME_INVALID_INDEX) {
            bfError << "Cannot find " + frame2 + " in the frame list.";
            return false;
        }
    }
    else {
        pImpl->frameIsCoM = true;
        pImpl->frameIndex2 = iDynTree::FRAME_INVALID_INDEX;
    }

    // Initialize buffers
    // ------------------

    // Get the DoFs
    const auto dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    pImpl->jacobianCOM.resize(3, dofs);
    pImpl->jacobianCOM.zero();

    // Output
    pImpl->jacobian.resize(6, dofs);
    pImpl->jacobian.zero();

    return true;
}

bool RelativeJacobian::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool RelativeJacobian::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    using MatrixXdSimulink = Matrix<double, Dynamic, Dynamic, Eigen::ColMajor>;
    using MatrixXdiDynTree = Matrix<double, Dynamic, Dynamic, Eigen::RowMajor>;

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

    // Compute the jacobian
    ok = false;
    if (!pImpl->frameIsCoM) {
        world_H_frame = kinDyn->getWorldTransform(pImpl->frameIndex1);
        ok = kinDyn->getRelativeJacobian(pImpl->frameIndex1, pImpl->frameIndex2, pImpl->jacobian);
    }
    else {
        world_H_frame.setPosition(kinDyn->getCenterOfMassPosition());
        // TODO check if there's the possibility to have the CoM jacobian relative to another frame
        ok = kinDyn->getCenterOfMassJacobian(pImpl->jacobianCOM);
        auto cols = pImpl->jacobianCOM.cols();
        toEigen(pImpl->jacobian).block(0, 0, 3, cols) = toEigen(pImpl->jacobianCOM);
        toEigen(pImpl->jacobian).block(3, 0, 3, cols).setZero();
    }

    if (!ok) {
        bfError << "Failed to get the Jacobian.";
        return false;
    }

    // Get the output signal memory location
    OutputSignalPtr output = blockInfo->getOutputPortSignal(OutputIndex::RelativeJacobian);
    if (!output) {
        bfError << "Output signal not valid.";
        return false;
    }

    // Allocate objects for row-major -> col-major conversion
    Map<MatrixXdiDynTree> jacobianRowMajor = toEigen(pImpl->jacobian);
    Map<MatrixXdSimulink> jacobianColMajor(
        output->getBuffer<double>(),
        blockInfo->getOutputPortMatrixSize(OutputIndex::RelativeJacobian).rows,
        blockInfo->getOutputPortMatrixSize(OutputIndex::RelativeJacobian).cols);

    // Forward the buffer to Simulink transforming it to ColMajor
    jacobianColMajor = jacobianRowMajor;
    return true;
}
