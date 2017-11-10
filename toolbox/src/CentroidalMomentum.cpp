#include "CentroidalMomentum.h"

#include "BlockInformation.h"
#include "Signal.h"
#include "Log.h"
#include "RobotInterface.h"
#include <memory>
#include <iDynTree/Core/SpatialMomentum.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <Eigen/Core>

using namespace wbt;

const std::string CentroidalMomentum::ClassName = "CentroidalMomentum";

const unsigned CentroidalMomentum::INPUT_IDX_BASE_POSE = 0;
const unsigned CentroidalMomentum::INPUT_IDX_JOINTCONF = 1;
const unsigned CentroidalMomentum::INPUT_IDX_BASE_VEL  = 2;
const unsigned CentroidalMomentum::INPUT_IDX_JOINT_VEL = 3;
const unsigned CentroidalMomentum::OUTPUT_IDX_CENTRMOM = 0;

CentroidalMomentum::CentroidalMomentum()
: m_centroidalMomentum(nullptr)
{}

bool CentroidalMomentum::configureSizeAndPorts(BlockInformation* blockInfo)
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
    blockInfo->setInputPortType(INPUT_IDX_BASE_VEL,  PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_JOINT_VEL, PortDataTypeDouble);

    if (!success) {
        Log::getSingleton().error("Failed to configure input ports.");
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Vector representing the centroidal momentum (1x6)
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        Log::getSingleton().error("Failed to configure the number of output ports.");
        return false;
    }

    // Size and type
    success = blockInfo->setOutputPortVectorSize(OUTPUT_IDX_CENTRMOM, 6);
    blockInfo->setOutputPortType(OUTPUT_IDX_CENTRMOM, PortDataTypeDouble);

    return success;
}

bool CentroidalMomentum::initialize(const BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) return false;

    // OUTPUT
    // ======

    m_centroidalMomentum = new iDynTree::SpatialMomentum();
    return m_centroidalMomentum;
}

bool CentroidalMomentum::terminate(const BlockInformation* blockInfo)
{
    if (m_centroidalMomentum) {
        delete m_centroidalMomentum;
        m_centroidalMomentum = 0;
    }

    return WBBlock::terminate(blockInfo);
}

bool CentroidalMomentum::output(const BlockInformation* blockInfo)
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

    // UPDATE THE ROBOT STATUS
    // =======================
    model->setRobotState(robotState.m_world_T_base,
                         robotState.m_jointsPosition,
                         robotState.m_baseVelocity,
                         robotState.m_jointsVelocity,
                         robotState.m_gravity);

    // OUTPUT
    // ======

    // Calculate the centroidal momentum
    *m_centroidalMomentum = model->getCentroidalTotalMomentum();

    // Forward the output to Simulink
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_CENTRMOM);
    output.setBuffer(toEigen(*m_centroidalMomentum).data(),
                     blockInfo->getOutputPortWidth(OUTPUT_IDX_CENTRMOM));
    return true;
}
