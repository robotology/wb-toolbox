#include "InverseDynamics.h"
#include "BlockInformation.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingState.h>

#include <memory>

using namespace wbt;

const std::string InverseDynamics::ClassName = "InverseDynamics";

const unsigned INPUT_IDX_BASE_POSE = 0;
const unsigned INPUT_IDX_JOINTCONF = 1;
const unsigned INPUT_IDX_BASE_VEL = 2;
const unsigned INPUT_IDX_JOINT_VEL = 3;
const unsigned INPUT_IDX_BASE_ACC = 4;
const unsigned INPUT_IDX_JOINT_ACC = 5;
const unsigned OUTPUT_IDX_TORQUES = 0;

InverseDynamics::InverseDynamics() {}

unsigned InverseDynamics::numberOfParameters()
{
    return WBBlock::numberOfParameters();
}

bool InverseDynamics::configureSizeAndPorts(BlockInformation* blockInfo)
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
    // 5) Base frame acceleration (1x6 vector)
    // 6) Joints acceleration (1xDoFs vector)
    //

    // Number of inputs
    if (!blockInfo->setNumberOfInputPorts(6)) {
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
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_BASE_ACC, 6);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINT_ACC, dofs);

    blockInfo->setInputPortType(INPUT_IDX_BASE_POSE, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINTCONF, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_BASE_VEL, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINT_VEL, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_BASE_ACC, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINT_ACC, DataType::DOUBLE);

    if (!success) {
        wbtError << "Failed to configure input ports.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Vector representing the torques (1x(DoFs+6))
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        wbtError << "Failed to configure the number of output ports.";
        return false;
    }

    // Size and type
    success = blockInfo->setOutputPortVectorSize(OUTPUT_IDX_TORQUES, dofs + 6);
    blockInfo->setOutputPortType(OUTPUT_IDX_TORQUES, DataType::DOUBLE);

    return success;
}

bool InverseDynamics::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // OUTPUT / VARIABLES
    // ==================

    using namespace iDynTree;

    // Get the DoFs
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }
    const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();

    m_baseAcceleration = std::unique_ptr<Vector6>(new Vector6());
    m_baseAcceleration->zero();
    m_jointsAcceleration = std::unique_ptr<VectorDynSize>(new VectorDynSize(dofs));
    m_jointsAcceleration->zero();

    // Get the KinDynComputations pointer
    const auto& kindyn = robotInterface->getKinDynComputations();
    if (!kindyn) {
        wbtError << "Failed to get the KinDynComputations object";
        return false;
    }

    // Get the model from the KinDynComputations object
    const auto& model = kindyn->model();

    m_torques =
        std::unique_ptr<FreeFloatingGeneralizedTorques>(new FreeFloatingGeneralizedTorques(model));

    return static_cast<bool>(m_baseAcceleration) && static_cast<bool>(m_jointsAcceleration)
           && static_cast<bool>(m_torques);
}

bool InverseDynamics::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool InverseDynamics::output(const BlockInformation* blockInfo)
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

    // Base acceleration
    // -----------------

    const Signal baseAccelerationSignal = blockInfo->getInputPortSignal(INPUT_IDX_BASE_ACC);
    if (!baseAccelerationSignal.isValid()) {
        wbtError << "Base Acceleration signal not valid.";
        return false;
    }
    double* bufBaseAcc = baseAccelerationSignal.getBuffer<double>();

    for (unsigned i = 0; i < baseAccelerationSignal.getWidth(); ++i) {
        if (!m_baseAcceleration->setVal(i, bufBaseAcc[i])) {
            wbtError << "Failed to fill base accelerations class member.";
            return false;
        }
    }

    // Joints acceleration
    // -------------------

    const Signal jointsAccelerationSignal = blockInfo->getInputPortSignal(INPUT_IDX_JOINT_ACC);
    if (!jointsAccelerationSignal.isValid()) {
        wbtError << "Joints Acceleration signal not valid.";
        return false;
    }
    double* bufJointsAcc = jointsAccelerationSignal.getBuffer<double>();

    for (unsigned i = 0; i < jointsAccelerationSignal.getWidth(); ++i) {
        if (!m_jointsAcceleration->setVal(i, bufJointsAcc[i])) {
            wbtError << "Failed to fill joint accelerations class member.";
            return false;
        }
    }

    // OUTPUT
    // ======

    // Calculate the inverse dynamics (assuming zero external forces)
    ok = kinDyn->inverseDynamics(*m_baseAcceleration,
                                 *m_jointsAcceleration,
                                 iDynTree::LinkNetExternalWrenches(kinDyn->getNrOfLinks()),
                                 *m_torques);

    if (!ok) {
        wbtError << "iDynTree failed to compute inverse dynamics.";
        return false;
    }

    // Get the output signal
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_TORQUES);
    if (!output.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }
    double* outputBuffer = output.getBuffer<double>();

    // Convert generalized torques and forward the directly to Simulink
    // mapping the memory through Eigen::Map
    const auto& torquesSize = m_torques->jointTorques().size();
    Eigen::Map<Eigen::VectorXd> generalizedOutputTrqs(outputBuffer, torquesSize + 6);
    generalizedOutputTrqs.segment(0, 6) = toEigen(m_torques->baseWrench());
    generalizedOutputTrqs.segment(6, torquesSize) = toEigen(m_torques->jointTorques());

    return true;
}
