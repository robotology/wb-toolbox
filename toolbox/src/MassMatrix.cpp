#include "MassMatrix.h"

#include "BlockInformation.h"
#include "Signal.h"
#include "Log.h"
#include "RobotInterface.h"
#include <memory>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <Eigen/Core>

using namespace wbt;

const std::string MassMatrix::ClassName = "MassMatrix";

const unsigned MassMatrix::INPUT_IDX_BASE_POSE = 0;
const unsigned MassMatrix::INPUT_IDX_JOINTCONF = 1;
const unsigned MassMatrix::OUTPUT_IDX_MASS_MAT = 0;

MassMatrix::MassMatrix()
: m_massMatrix(nullptr)
{}

bool MassMatrix::configureSizeAndPorts(BlockInformation* blockInfo)
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
    //

    // Number of inputs
    if (!blockInfo->setNumberOfInputPorts(2)) {
        Log::getSingleton().error("Failed to configure the number of input ports.");
        return false;
    }

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Size and type
    bool success = true;
    success = success && blockInfo->setInputPortMatrixSize(INPUT_IDX_BASE_POSE, 4, 4);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINTCONF, dofs);

    blockInfo->setInputPortType(INPUT_IDX_BASE_POSE, PortDataTypeDouble);
    blockInfo->setInputPortType(INPUT_IDX_JOINTCONF, PortDataTypeDouble);

    if (!success) {
        Log::getSingleton().error("Failed to configure input ports.");
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Matrix epresenting the mass matrix (DoFs+6)x(DoFs+6)
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        Log::getSingleton().error("Failed to configure the number of output ports.");
        return false;
    }

    // Size and type
    success = blockInfo->setOutputPortMatrixSize(OUTPUT_IDX_MASS_MAT, dofs + 6, dofs + 6);
    blockInfo->setOutputPortType(OUTPUT_IDX_MASS_MAT, PortDataTypeDouble);

    return success;
}

bool MassMatrix::initialize(const BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) return false;

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Output
    m_massMatrix = new iDynTree::MatrixDynSize(6 + dofs, 6 + dofs);
    m_massMatrix->zero();

    return m_massMatrix;
}

bool MassMatrix::terminate(const BlockInformation* blockInfo)
{
    if (m_massMatrix) {
        delete m_massMatrix;
        m_massMatrix = nullptr;
    }

    return WBBlock::terminate(blockInfo);
}

bool MassMatrix::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    using namespace iDynTree;
    typedef Matrix<double, 4, 4, ColMajor> Matrix4dSimulink;
    typedef Matrix<double, Dynamic, Dynamic, ColMajor> MatrixXdSimulink;
    typedef Matrix<double, Dynamic, Dynamic, Eigen::RowMajor> MatrixXdiDynTree;

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

    // UPDATE THE ROBOT STATUS
    // =======================
    model->setRobotState(robotState.m_world_T_base,
                         robotState.m_jointsPosition,
                         robotState.m_baseVelocity,
                         robotState.m_jointsVelocity,
                         robotState.m_gravity);

    // OUTPUT
    // ======

    // Compute the Mass Matrix
    model->getFreeFloatingMassMatrix(*m_massMatrix);

    // Get the output signal memory location
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_MASS_MAT);
    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Allocate objects for row-major -> col-major conversion
    Map<MatrixXdiDynTree> massMatrixRowMajor = toEigen(*m_massMatrix);
    Map<MatrixXdSimulink> massMatrixColMajor((double*)output.getContiguousBuffer(),
                                             6 + dofs, 6 + dofs);

    // Forward the buffer to Simulink transforming it to ColMajor
    massMatrixColMajor = massMatrixRowMajor;
    return true;
}
