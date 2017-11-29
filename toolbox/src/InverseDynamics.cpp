#include "InverseDynamics.h"

#include "Log.h"
#include "BlockInformation.h"
#include "Signal.h"
#include "RobotInterface.h"
#include <memory>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>

using namespace wbt;

const std::string InverseDynamics::ClassName = "InverseDynamics";

const unsigned InverseDynamics::INPUT_IDX_BASE_POSE = 0;
const unsigned InverseDynamics::INPUT_IDX_JOINTCONF = 1;
const unsigned InverseDynamics::INPUT_IDX_BASE_VEL  = 2;
const unsigned InverseDynamics::INPUT_IDX_JOINT_VEL = 3;
const unsigned InverseDynamics::INPUT_IDX_BASE_ACC  = 4;
const unsigned InverseDynamics::INPUT_IDX_JOINT_ACC = 5;
const unsigned InverseDynamics::OUTPUT_IDX_TORQUES  = 0;

InverseDynamics::InverseDynamics() {}

unsigned InverseDynamics::numberOfParameters()
{
    return WBBlock::numberOfParameters();
}

bool InverseDynamics::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // Memory allocation / Saving data not allowed here

    if (!WBBlock::configureSizeAndPorts(blockInfo)) return false;

    // INPUTS
    // ======
    //
    // 1) Homogeneous transform for base pose wrt the world frame (4x4 matrix)
    // 2) Joints position (1xDoFs vector)
    // 3) Base frame velocity (1x6 vector)
    // 4) Joints velocity (1xDoFs vector)
    // 5) Base frame acceleration (1x6 vector)
    // 6) Joints acceleration (1xDoFs vector)
    //

    // Number of inputs
    if (!blockInfo->setNumberOfInputPorts(6)) {
        Log::getSingleton().error("Failed to configure the number of input ports.");
        return false;
    }

    // Get the DoFs
    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Size and type
    bool success = true;
    success = success && blockInfo->setInputPortMatrixSize(INPUT_IDX_BASE_POSE, 4, 4);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINTCONF, dofs);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_BASE_VEL,  6);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINT_VEL, dofs);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_BASE_ACC,  6);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINT_ACC, dofs);

    blockInfo->setInputPortType(INPUT_IDX_BASE_POSE, PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_JOINTCONF, PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_BASE_VEL,  PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_JOINT_VEL, PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_BASE_ACC,  PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_JOINT_ACC, PortDataTypeDouble);

    if (!success) {
        Log::getSingleton().error("Failed to configure input ports.");
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Vector representing the torques (1x(DoFs+6))
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        Log::getSingleton().error("Failed to configure the number of output ports.");
        return false;
    }

    // Size and type
    success = blockInfo->setOutputPortVectorSize(OUTPUT_IDX_TORQUES, dofs + 6);
    blockInfo->setOutputPortType(OUTPUT_IDX_TORQUES, PortDataTypeDouble);

    return success;
}

bool InverseDynamics::initialize(const BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) return false;

    // OUTPUT / VARIABLES
    // ==================

    using namespace iDynTree;
    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    m_baseAcceleration = std::unique_ptr<Vector6>(new Vector6());
    m_baseAcceleration->zero();
    m_jointsAcceleration = std::unique_ptr<VectorDynSize>(new VectorDynSize(dofs));
    m_jointsAcceleration->zero();

    // Get the KinDynComputations pointer
    const auto& kindyncomp = getRobotInterface()->getKinDynComputations();

    // Get the model from the KinDynComputations object
    const auto& model = kindyncomp->model();

    m_torques = std::unique_ptr<FreeFloatingGeneralizedTorques>(new FreeFloatingGeneralizedTorques(model));

    return static_cast<bool>(m_baseAcceleration) &&
           static_cast<bool>(m_jointsAcceleration) &&
           static_cast<bool>(m_torques);
}

bool InverseDynamics::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool InverseDynamics::output(const BlockInformation* blockInfo)
{
    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    Signal basePoseSig = blockInfo->getInputPortSignal(INPUT_IDX_BASE_POSE);
    Signal jointsPosSig = blockInfo->getInputPortSignal(INPUT_IDX_JOINTCONF);
    Signal baseVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_BASE_VEL);
    Signal jointsVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_JOINT_VEL);

    bool ok = setRobotState(&basePoseSig,
                            &jointsPosSig,
                            &baseVelocitySignal,
                            &jointsVelocitySignal);

    if (!ok) {
        Log::getSingleton().error("Failed to set the robot state.");
        return false;
    }

    // Base acceleration
    // -----------------

    Signal baseAccelerationSignal = blockInfo->getInputPortSignal(INPUT_IDX_BASE_ACC);
    double* bufBaseAcc = baseAccelerationSignal.getBuffer<double>();
    if (!bufBaseAcc) {
        Log::getSingleton().error("Failed to read data from input port.");
        return false;
    }
    for (auto i = 0; i < baseAccelerationSignal.getWidth(); ++i) {
        m_baseAcceleration->setVal(i, bufBaseAcc[i]);
    }

    // Joints acceleration
    // -------------------

    Signal jointsAccelerationSignal = blockInfo->getInputPortSignal(INPUT_IDX_JOINT_ACC);
    double* bufJointsAcc = jointsAccelerationSignal.getBuffer<double>();
    if (!bufJointsAcc) {
        Log::getSingleton().error("Failed to read data from input port.");
        return false;
    }
    for (auto i = 0; i < jointsAccelerationSignal.getWidth(); ++i) {
        m_jointsAcceleration->setVal(i, bufJointsAcc[i]);
    }

    // OUTPUT
    // ======

    const auto& model = getRobotInterface()->getKinDynComputations();

    if (!model) {
        Log::getSingleton().error("Failed to retrieve the KinDynComputations object.");
        return false;
    }

    // Calculate the inverse dynamics (assuming zero external forces)
    model->inverseDynamics(*m_baseAcceleration,
                           *m_jointsAcceleration,
                           iDynTree::LinkNetExternalWrenches(model->getNrOfLinks()),
                           *m_torques);

    // Forward the output to Simulink
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_TORQUES);
    output.setBuffer(m_torques->jointTorques().data(),
                     blockInfo->getOutputPortWidth(OUTPUT_IDX_TORQUES));
    return true;
}
