/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "Jacobian.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixDynSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Indices.h>

#include <memory>
#include <ostream>
#include <tuple>

using namespace wbt;
const std::string Jacobian::ClassName = "Jacobian";

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = WBBlock::NumberOfParameters - 1,
    Frame
};

enum InputIndex
{
    BasePose = 0,
    JointConfiguration,
};

enum OutputIndex
{
    Jacobian = 0,
};

// BLOCK PIMPL
// ===========

class Jacobian::impl
{
public:
    iDynTree::MatrixDynSize jacobianCOM;
    iDynTree::MatrixDynSize jacobian;

    bool frameIsCoM = false;
    iDynTree::FrameIndex frameIndex = iDynTree::FRAME_INVALID_INDEX;
};

// BLOCK CLASS
// ===========

Jacobian::Jacobian()
    : pImpl{new impl()}
{}

unsigned Jacobian::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool Jacobian::parseParameters(BlockInformation* blockInfo)
{
    const ParameterMetadata frameMetadata(ParameterType::STRING, ParamIndex::Frame, 1, 1, "Frame");

    bool ok = blockInfo->addParameterMetadata(frameMetadata);

    if (!ok) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool Jacobian::configureSizeAndPorts(BlockInformation* blockInfo)
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
    // 1) Matrix representing the Jacobian (6x(DoFs+6))
    //

    const bool ok = blockInfo->setIOPortsData({
        {
            // Inputs
            std::make_tuple(InputIndex::BasePose, std::vector<int>{4, 4}, DataType::DOUBLE),
            std::make_tuple(
                InputIndex::JointConfiguration, std::vector<int>{dofs}, DataType::DOUBLE),
        },
        {
            // Outputs
            std::make_tuple(OutputIndex::Jacobian, std::vector<int>{6, 6 + dofs}, DataType::DOUBLE),
        },
    });

    if (!ok) {
        wbtError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool Jacobian::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!Jacobian::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    std::string frame;
    if (!m_parameters.getParameter("Frame", frame)) {
        wbtError << "Cannot retrieve string from frame parameter.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    // Check if the frame is valid
    // ---------------------------

    auto kinDyn = getKinDynComputations();
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

    // Initialize buffers
    // ------------------

    // Get the DoFs
    const auto dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    pImpl->jacobianCOM.resize(3, 6 + dofs);
    pImpl->jacobianCOM.zero();

    // Output
    pImpl->jacobian.resize(6, 6 + dofs);
    pImpl->jacobian.zero();

    return true;
}

bool Jacobian::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool Jacobian::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    using MatrixXdSimulink = Matrix<double, Dynamic, Dynamic, Eigen::ColMajor>;
    using MatrixXdiDynTree = Matrix<double, Dynamic, Dynamic, Eigen::RowMajor>;

    // Get the KinDynComputations object
    auto kinDyn = getKinDynComputations();
    if (!kinDyn) {
        wbtError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    InputSignalPtr basePoseSig = blockInfo->getInputPortSignal(InputIndex::BasePose);
    InputSignalPtr jointsPosSig = blockInfo->getInputPortSignal(InputIndex::JointConfiguration);

    if (!basePoseSig || !jointsPosSig) {
        wbtError << "Input signals not valid.";
        return false;
    }

    bool ok = setRobotState(basePoseSig, jointsPosSig, nullptr, nullptr, kinDyn.get());

    if (!ok) {
        wbtError << "Failed to set the robot state.";
        return false;
    }

    // OUTPUT
    // ======

    iDynTree::Transform world_H_frame;

    // Compute the jacobian
    ok = false;
    if (!pImpl->frameIsCoM) {
        world_H_frame = kinDyn->getWorldTransform(pImpl->frameIndex);
        ok = kinDyn->getFrameFreeFloatingJacobian(pImpl->frameIndex, pImpl->jacobian);
    }
    else {
        world_H_frame.setPosition(kinDyn->getCenterOfMassPosition());
        ok = kinDyn->getCenterOfMassJacobian(pImpl->jacobianCOM);
        auto cols = pImpl->jacobianCOM.cols();
        toEigen(pImpl->jacobian).block(0, 0, 3, cols) = toEigen(pImpl->jacobianCOM);
        toEigen(pImpl->jacobian).block(3, 0, 3, cols).setZero();
    }

    if (!ok) {
        wbtError << "Failed to get the Jacobian.";
        return false;
    }

    // Get the output signal memory location
    OutputSignalPtr output = blockInfo->getOutputPortSignal(OutputIndex::Jacobian);
    if (!output) {
        wbtError << "Output signal not valid.";
        return false;
    }

    // Allocate objects for row-major -> col-major conversion
    Map<MatrixXdiDynTree> jacobianRowMajor = toEigen(pImpl->jacobian);
    Map<MatrixXdSimulink> jacobianColMajor(
        output->getBuffer<double>(),
        blockInfo->getOutputPortMatrixSize(OutputIndex::Jacobian).first,
        blockInfo->getOutputPortMatrixSize(OutputIndex::Jacobian).second);

    // Forward the buffer to Simulink transforming it to ColMajor
    jacobianColMajor = jacobianRowMajor;
    return true;
}
