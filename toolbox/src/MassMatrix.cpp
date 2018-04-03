#include "MassMatrix.h"
#include "BlockInformation.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>

#include <memory>

using namespace wbt;

const std::string MassMatrix::ClassName = "MassMatrix";

const unsigned INPUT_IDX_BASE_POSE = 0;
const unsigned INPUT_IDX_JOINTCONF = 1;
const unsigned OUTPUT_IDX_MASS_MAT = 0;

MassMatrix::MassMatrix() {}

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
        wbtError << "Failed to configure the number of input ports.";
        return false;
    }

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Size and type
    bool success = true;
    success = success && blockInfo->setInputPortMatrixSize(INPUT_IDX_BASE_POSE, 4, 4);
    success = success && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINTCONF, dofs);

    blockInfo->setInputPortType(INPUT_IDX_BASE_POSE, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINTCONF, DataType::DOUBLE);

    if (!success) {
        wbtError << "Failed to configure input ports.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Matrix epresenting the mass matrix (DoFs+6)x(DoFs+6)
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        wbtError << "Failed to configure the number of output ports.";
        return false;
    }

    // Size and type
    success = blockInfo->setOutputPortMatrixSize(OUTPUT_IDX_MASS_MAT, dofs + 6, dofs + 6);
    blockInfo->setOutputPortType(OUTPUT_IDX_MASS_MAT, DataType::DOUBLE);

    return success;
}

bool MassMatrix::initialize(const BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Output
    m_massMatrix =
        std::unique_ptr<iDynTree::MatrixDynSize>(new iDynTree::MatrixDynSize(6 + dofs, 6 + dofs));
    m_massMatrix->zero();

    return static_cast<bool>(m_massMatrix);
}

bool MassMatrix::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool MassMatrix::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    using namespace iDynTree;
    typedef Matrix<double, Dynamic, Dynamic, ColMajor> MatrixXdSimulink;
    typedef Matrix<double, Dynamic, Dynamic, Eigen::RowMajor> MatrixXdiDynTree;

    const auto& model = getRobotInterface()->getKinDynComputations();

    if (!model) {
        wbtError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    Signal basePoseSig = blockInfo->getInputPortSignal(INPUT_IDX_BASE_POSE);
    Signal jointsPosSig = blockInfo->getInputPortSignal(INPUT_IDX_JOINTCONF);

    bool ok = setRobotState(&basePoseSig, &jointsPosSig, nullptr, nullptr);

    if (!ok) {
        wbtError << "Failed to set the robot state.";
        return false;
    }

    // OUTPUT
    // ======

    // Compute the Mass Matrix
    model->getFreeFloatingMassMatrix(*m_massMatrix);

    // Get the output signal memory location
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_MASS_MAT);
    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Allocate objects for row-major -> col-major conversion
    Map<MatrixXdiDynTree> massMatrixRowMajor = toEigen(*m_massMatrix);
    Map<MatrixXdSimulink> massMatrixColMajor(output.getBuffer<double>(), 6 + dofs, 6 + dofs);

    // Forward the buffer to Simulink transforming it to ColMajor
    massMatrixColMajor = massMatrixRowMajor;
    return true;
}
