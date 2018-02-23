#include "DotJNu.h"
#include "BlockInformation.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>

#include <memory>

using namespace wbt;

const std::string DotJNu::ClassName = "DotJNu";

const unsigned DotJNu::INPUT_IDX_BASE_POSE = 0;
const unsigned DotJNu::INPUT_IDX_JOINTCONF = 1;
const unsigned DotJNu::INPUT_IDX_BASE_VEL = 2;
const unsigned DotJNu::INPUT_IDX_JOINT_VEL = 3;
const unsigned DotJNu::OUTPUT_IDX_DOTJ_NU = 0;

DotJNu::DotJNu()
    : m_frameIsCoM(false)
    , m_frameIndex(iDynTree::FRAME_INVALID_INDEX)
{}

unsigned DotJNu::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool DotJNu::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // Memory allocation / Saving data not allowed here

    if (!WBBlock::configureSizeAndPorts(blockInfo))
        return false;

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
        Log::getSingleton().error("Failed to configure the number of input ports.");
        return false;
    }

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Size and type
    bool success = true;
    success = success && blockInfo->setInputPortMatrixSize(INPUT_IDX_BASE_POSE, 4, 4);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINTCONF, dofs);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_BASE_VEL, 6);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINT_VEL, dofs);

    blockInfo->setInputPortType(INPUT_IDX_BASE_POSE, PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_JOINTCONF, PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_BASE_VEL, PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_JOINT_VEL, PortDataTypeDouble);

    if (!success) {
        Log::getSingleton().error("Failed to configure input ports.");
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Vector representing the \dot{J} \nu vector (1x6)
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        Log::getSingleton().error("Failed to configure the number of output ports.");
        return false;
    }

    // Size and type
    success = blockInfo->setOutputPortVectorSize(OUTPUT_IDX_DOTJ_NU, 6);
    blockInfo->setOutputPortType(OUTPUT_IDX_DOTJ_NU, PortDataTypeDouble);

    return success;
}

bool DotJNu::initialize(const BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // INPUT PARAMETERS
    // ================

    std::string frame;
    int parentParameters = WBBlock::numberOfParameters();

    if (!blockInfo->getStringParameterAtIndex(parentParameters + 1, frame)) {
        Log::getSingleton().error("Cannot retrieve string from frame parameter.");
        return false;
    }

    // Check if the frame is valid
    // ---------------------------

    const auto& model = getRobotInterface()->getKinDynComputations();
    if (!model) {
        Log::getSingleton().error("Cannot retrieve handle to KinDynComputations.");
        return false;
    }

    if (frame != "com") {
        m_frameIndex = model->getFrameIndex(frame);
        if (m_frameIndex == iDynTree::FRAME_INVALID_INDEX) {
            Log::getSingleton().error("Cannot find " + frame + " in the frame list.");
            return false;
        }
    }
    else {
        m_frameIsCoM = true;
        m_frameIndex = iDynTree::FRAME_INVALID_INDEX;
    }

    // OUTPUT
    // ======
    m_dotJNu = std::unique_ptr<iDynTree::Vector6>(new iDynTree::Vector6());
    m_dotJNu->zero();

    return static_cast<bool>(m_dotJNu);
}

bool DotJNu::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool DotJNu::output(const BlockInformation* blockInfo)
{
    const auto& model = getRobotInterface()->getKinDynComputations();

    if (!model) {
        Log::getSingleton().error("Failed to retrieve the KinDynComputations object.");
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    Signal basePoseSig = blockInfo->getInputPortSignal(INPUT_IDX_BASE_POSE);
    Signal jointsPosSig = blockInfo->getInputPortSignal(INPUT_IDX_JOINTCONF);
    Signal baseVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_BASE_VEL);
    Signal jointsVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_JOINT_VEL);

    bool ok =
        setRobotState(&basePoseSig, &jointsPosSig, &baseVelocitySignal, &jointsVelocitySignal);

    if (!ok) {
        Log::getSingleton().error("Failed to set the robot state.");
        return false;
    }

    // OUTPUT
    // ======

    if (!m_frameIsCoM) {
        *m_dotJNu = model->getFrameBiasAcc(m_frameIndex);
    }
    else {
        iDynTree::Vector3 comBiasAcc = model->getCenterOfMassBiasAcc();
        toEigen(*m_dotJNu).segment<3>(0) = iDynTree::toEigen(comBiasAcc);
        toEigen(*m_dotJNu).segment<3>(3).setZero();
    }

    // Forward the output to Simulink
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_DOTJ_NU);
    output.setBuffer(m_dotJNu->data(), blockInfo->getOutputPortWidth(OUTPUT_IDX_DOTJ_NU));
    return true;
}
