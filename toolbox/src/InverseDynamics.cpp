#include "InverseDynamics.h"

#include "Log.h"
#include "BlockInformation.h"
#include "Signal.h"
#include "RobotInterface.h"
#include <memory>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <Eigen/Core>

using namespace wbt;

const std::string InverseDynamics::ClassName = "InverseDynamics";

const unsigned InverseDynamics::INPUT_IDX_BASE_POSE = 0;
const unsigned InverseDynamics::INPUT_IDX_JOINTCONF = 1;
const unsigned InverseDynamics::INPUT_IDX_BASE_VEL  = 2;
const unsigned InverseDynamics::INPUT_IDX_JOINT_VEL = 3;
const unsigned InverseDynamics::INPUT_IDX_BASE_ACC  = 4;
const unsigned InverseDynamics::INPUT_IDX_JOINT_ACC = 5;
const unsigned InverseDynamics::OUTPUT_IDX_TORQUES  = 0;

InverseDynamics::InverseDynamics()
: m_baseAcceleration(nullptr)
, m_jointsAcceleration(nullptr)
, m_torques(nullptr)
{}

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

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    m_baseAcceleration = new iDynTree::Vector6();
    m_baseAcceleration->zero();
    m_jointsAcceleration = new iDynTree::VectorDynSize(dofs);
    m_jointsAcceleration->zero();

    const auto& model = getRobotInterface()->getKinDynComputations()->model();
    m_torques = new iDynTree::FreeFloatingGeneralizedTorques(model);

    return m_baseAcceleration && m_jointsAcceleration && m_torques;
}

bool InverseDynamics::terminate(const BlockInformation* blockInfo)
{
    if (m_baseAcceleration) {
        delete m_baseAcceleration;
        m_baseAcceleration = nullptr;
    }
    if (m_jointsAcceleration) {
        delete m_jointsAcceleration;
        m_jointsAcceleration = nullptr;
    }
    if (m_torques) {
        delete m_torques;
        m_torques = nullptr;
    }

    return WBBlock::terminate(blockInfo);
}

bool InverseDynamics::output(const BlockInformation* blockInfo)
{
    using namespace iDynTree;
    using namespace Eigen;
    typedef Matrix<double, 4, 4, ColMajor> Matrix4dSimulink;

    const auto& model = getRobotInterface()->getKinDynComputations();

    if (!model) {
        Log::getSingleton().error("Failed to retrieve the KinDynComputations object.");
        return false;
    }

    // GET THE SIGNALS AND CONVERT THEM TO IDYNTREE OBJECTS
    // ====================================================

    unsigned signalWidth;

    // Base pose
    Signal basePoseSig = blockInfo->getInputPortSignal(INPUT_IDX_BASE_POSE);
    signalWidth = blockInfo->getInputPortWidth(INPUT_IDX_BASE_POSE);
    fromEigen(robotState.m_world_T_base,
              Matrix4dSimulink(basePoseSig.getStdVector(signalWidth).data()));

    // Joints position
    Signal jointsPositionSig = blockInfo->getInputPortSignal(INPUT_IDX_JOINTCONF);
    signalWidth = blockInfo->getInputPortWidth(INPUT_IDX_JOINTCONF);
    robotState.m_jointsPosition.fillBuffer(jointsPositionSig.getStdVector(signalWidth).data());

    // Base velocity
    Signal baseVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_BASE_VEL);
    signalWidth = blockInfo->getInputPortWidth(INPUT_IDX_BASE_VEL);
    double* m_baseVelocityBuffer = baseVelocitySignal.getStdVector(signalWidth).data();
    robotState.m_baseVelocity = Twist(LinVelocity(m_baseVelocityBuffer, 3),
                                 AngVelocity(m_baseVelocityBuffer+3, 3));

    // Joints velocity
    Signal jointsVelocitySignal = blockInfo->getInputPortSignal(INPUT_IDX_JOINT_VEL);
    signalWidth = blockInfo->getInputPortWidth(INPUT_IDX_JOINT_VEL);
    robotState.m_jointsVelocity.fillBuffer(jointsVelocitySignal.getStdVector(signalWidth).data());

    // Base acceleration
    Signal baseAccelerationSignal = blockInfo->getInputPortSignal(INPUT_IDX_BASE_ACC);
    signalWidth = blockInfo->getInputPortWidth(INPUT_IDX_BASE_ACC);
    m_baseAcceleration->fillBuffer(baseAccelerationSignal.getStdVector(signalWidth).data());

    // Joints acceleration
    Signal jointsAccelerationSignal = blockInfo->getInputPortSignal(INPUT_IDX_JOINT_ACC);
    signalWidth = blockInfo->getInputPortWidth(INPUT_IDX_JOINT_ACC);
    m_jointsAcceleration->fillBuffer(jointsAccelerationSignal.getStdVector(signalWidth).data());

    // UPDATE THE ROBOT STATUS
    // =======================
    model->setRobotState(robotState.m_world_T_base,
                         robotState.m_jointsPosition,
                         robotState.m_baseVelocity,
                         robotState.m_jointsVelocity,
                         robotState.m_gravity);

    // OUTPUT
    // ======

    // Calculate the inverse dynamics (assuming zero external forces)
    model->inverseDynamics(*m_baseAcceleration,
                           *m_jointsAcceleration,
                           LinkNetExternalWrenches(model->getNrOfLinks()), // TODO
                           *m_torques);

    // Forward the output to Simulink
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_TORQUES);
    output.setBuffer(m_torques->jointTorques().data(),
                     blockInfo->getOutputPortWidth(OUTPUT_IDX_TORQUES));
    return true;
}
