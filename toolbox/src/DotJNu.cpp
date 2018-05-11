/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "DotJNu.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Indices.h>

#include <memory>
#include <ostream>
#include <tuple>

using namespace wbt;
const std::string DotJNu::ClassName = "DotJNu";

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
    BaseVelocity,
    JointVelocity,
};

enum OutputIndex
{
    DotJNu = 0,
};

// BLOCK PIMPL
// ===========

class DotJNu::impl
{
public:
    iDynTree::Vector6 dotJNu;
    bool frameIsCoM = false;
    iDynTree::FrameIndex frameIndex = iDynTree::FRAME_INVALID_INDEX;
};

// BLOCK CLASS
// ===========

DotJNu::DotJNu()
    : pImpl{new impl()}
{}

unsigned DotJNu::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool DotJNu::parseParameters(BlockInformation* blockInfo)
{
    const ParameterMetadata frameMetadata(ParameterType::STRING, ParamIndex::Frame, 1, 1, "Frame");

    if (!blockInfo->addParameterMetadata(frameMetadata)) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool DotJNu::configureSizeAndPorts(BlockInformation* blockInfo)
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
    // 1) Vector representing the \dot{J} \nu vector (1x6)
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
            std::make_tuple(OutputIndex::DotJNu, std::vector<int>{6}, DataType::DOUBLE),
        },
    });

    if (!ok) {
        wbtError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool DotJNu::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!DotJNu::parseParameters(blockInfo)) {
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

    // Initialize buffers
    // ------------------
    pImpl->dotJNu.zero();

    return true;
}

bool DotJNu::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool DotJNu::output(const BlockInformation* blockInfo)
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

    if (!pImpl->frameIsCoM) {
        pImpl->dotJNu = kinDyn->getFrameBiasAcc(pImpl->frameIndex);
    }
    else {
        iDynTree::Vector3 comBiasAcc = kinDyn->getCenterOfMassBiasAcc();
        toEigen(pImpl->dotJNu).segment<3>(0) = iDynTree::toEigen(comBiasAcc);
        toEigen(pImpl->dotJNu).segment<3>(3).setZero();
    }

    // Forward the output to Simulink
    Signal output = blockInfo->getOutputPortSignal(OutputIndex::DotJNu);
    if (!output.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }

    if (!output.setBuffer(pImpl->dotJNu.data(), output.getWidth())) {
        wbtError << "Failed to set output buffer.";
    }

    return true;
}
