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

using namespace wbt;

const std::string DotJNu::ClassName = "DotJNu";

const unsigned INPUT_IDX_BASE_POSE = 0;
const unsigned INPUT_IDX_JOINTCONF = 1;
const unsigned INPUT_IDX_BASE_VEL = 2;
const unsigned INPUT_IDX_JOINT_VEL = 3;
const unsigned OUTPUT_IDX_DOTJ_NU = 0;

const unsigned PARAM_IDX_BIAS = WBBlock::NumberOfParameters - 1;
const unsigned PARAM_IDX_FRAME = PARAM_IDX_BIAS + 1;

class DotJNu::impl
{
public:
    iDynTree::Vector6 dotJNu;
    bool frameIsCoM = false;
    iDynTree::FrameIndex frameIndex = iDynTree::FRAME_INVALID_INDEX;
};

DotJNu::DotJNu()
    : pImpl{new impl()}
{}

unsigned DotJNu::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool DotJNu::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata frameMetadata(ParameterType::STRING, PARAM_IDX_FRAME, 1, 1, "Frame");

    if (!blockInfo->addParameterMetadata(frameMetadata)) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool DotJNu::configureSizeAndPorts(BlockInformation* blockInfo)
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
    bool ok = true;
    ok = ok && blockInfo->setInputPortMatrixSize(INPUT_IDX_BASE_POSE, {4, 4});
    ok = ok && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINTCONF, dofs);
    ok = ok && blockInfo->setInputPortVectorSize(INPUT_IDX_BASE_VEL, 6);
    ok = ok && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINT_VEL, dofs);

    blockInfo->setInputPortType(INPUT_IDX_BASE_POSE, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINTCONF, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_BASE_VEL, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINT_VEL, DataType::DOUBLE);

    if (!ok) {
        wbtError << "Failed to configure input ports.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Vector representing the \dot{J} \nu vector (1x6)
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        wbtError << "Failed to configure the number of output ports.";
        return false;
    }

    // Size and type
    ok = blockInfo->setOutputPortVectorSize(OUTPUT_IDX_DOTJ_NU, 6);
    blockInfo->setOutputPortType(OUTPUT_IDX_DOTJ_NU, DataType::DOUBLE);

    return ok;
}

bool DotJNu::initialize(BlockInformation* blockInfo)
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

    // OUTPUT
    // ======
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

    const Signal basePoseSig = blockInfo->getInputPortSignal(INPUT_IDX_BASE_POSE);
    const Signal jointsPosSig = blockInfo->getInputPortSignal(INPUT_IDX_JOINTCONF);
    const Signal baseVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_BASE_VEL);
    const Signal jointsVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_JOINT_VEL);

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
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_DOTJ_NU);
    if (!output.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }

    if (!output.setBuffer(pImpl->dotJNu.data(), output.getWidth())) {
        wbtError << "Failed to set output buffer.";
    }

    return true;
}
