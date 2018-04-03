#include "Jacobian.h"
#include "BlockInformation.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>

#include <memory>

using namespace wbt;

const std::string Jacobian::ClassName = "Jacobian";

const unsigned INPUT_IDX_BASE_POSE = 0;
const unsigned INPUT_IDX_JOINTCONF = 1;
const unsigned OUTPUT_IDX_FW_FRAME = 0;

const unsigned PARAM_IDX_BIAS = WBBlock::NumberOfParameters - 1;
const unsigned PARAM_IDX_FRAME = PARAM_IDX_BIAS + 1;

Jacobian::Jacobian()
    : m_frameIsCoM(false)
    , m_frameIndex(iDynTree::FRAME_INVALID_INDEX)
{}

unsigned Jacobian::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool Jacobian::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // Memory allocation / Saving data not allowed here

    if (!WBBlock::configureSizeAndPorts(blockInfo)){
        return false;}

    // INPUTS
    // ======
    //
    // 1) Homogeneous transform for base pose wrt the world frame (4x4 matrix)
    // 2) Joints position (1xDoFs vector)
    // 3) Base frame velocity (1x6 vector)
    //

    // Number of inputs
    if (!blockInfo->setNumberOfInputPorts(2)) {
        wbtError << "Failed to configure the number of input ports.";
        return false;
    }

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Size and type
    bool ok = true;
    ok = ok && blockInfo->setInputPortMatrixSize(INPUT_IDX_BASE_POSE, {4, 4});
    ok = ok && blockInfo->setInputPortVectorSize(INPUT_IDX_JOINTCONF, dofs);

    blockInfo->setInputPortType(INPUT_IDX_BASE_POSE, DataType::DOUBLE);
    blockInfo->setInputPortType(INPUT_IDX_JOINTCONF, DataType::DOUBLE);

    if (!ok) {
        wbtError << "Failed to configure input ports.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // 1) Matrix representing the Jacobian (6x(DoFs+6))
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        wbtError << "Failed to configure the number of output ports.";
        return false;
    }

    // Size and type
    ok = blockInfo->setOutputPortMatrixSize(OUTPUT_IDX_FW_FRAME, {6, 6 + dofs});
    blockInfo->setOutputPortType(OUTPUT_IDX_FW_FRAME, DataType::DOUBLE);

    return ok;
}

bool Jacobian::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // INPUT PARAMETERS
    // ================

    std::string frame;
    int parentParameters = WBBlock::numberOfParameters();

    if (!blockInfo->getStringParameterAtIndex(parentParameters + 1, frame)) {
        wbtError << "Cannot retrieve string from frame parameter.";
        return false;
    }

    // Check if the frame is valid
    // ---------------------------

    const auto& model = getRobotInterface()->getKinDynComputations();
    if (!model) {
        wbtError << "Cannot retrieve handle to KinDynComputations.";
        return false;
    }

    if (frame != "com") {
        m_frameIndex = model->getFrameIndex(frame);
        if (m_frameIndex == iDynTree::FRAME_INVALID_INDEX) {
            wbtError << "Cannot find " + frame + " in the frame list.";
            return false;
        }
    }
    else {
        m_frameIsCoM = true;
        m_frameIndex = iDynTree::FRAME_INVALID_INDEX;
    }

    // OUTPUT / VARIABLES
    // ==================

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    m_jacobianCOM =
        std::unique_ptr<iDynTree::MatrixDynSize>(new iDynTree::MatrixDynSize(3, 6 + dofs));
    m_jacobianCOM->zero();

    // Output
    m_jacobian = std::unique_ptr<iDynTree::MatrixDynSize>(new iDynTree::MatrixDynSize(6, 6 + dofs));
    m_jacobian->zero();

    return static_cast<bool>(m_jacobianCOM) && static_cast<bool>(m_jacobian);
}

bool Jacobian::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool Jacobian::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    typedef Matrix<double, Dynamic, Dynamic, ColMajor> MatrixXdSimulink;
    typedef Matrix<double, Dynamic, Dynamic, Eigen::RowMajor> MatrixXdiDynTree;

    const auto& model = getRobotInterface()->getKinDynComputations();

    if (!model) {
        wbtError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    const Signal basePoseSig = blockInfo->getInputPortSignal(INPUT_IDX_BASE_POSE);
    const Signal jointsPosSig = blockInfo->getInputPortSignal(INPUT_IDX_JOINTCONF);

    bool ok = setRobotState(&basePoseSig, &jointsPosSig, nullptr, nullptr);

    if (!ok) {
        wbtError << "Failed to set the robot state.";
        return false;
    }

    // OUTPUT
    // ======

    iDynTree::Transform world_H_frame;

    // Compute the jacobian
    ok = false;
    if (!m_frameIsCoM) {
        world_H_frame = model->getWorldTransform(m_frameIndex);
        ok = model->getFrameFreeFloatingJacobian(m_frameIndex, *m_jacobian);
    }
    else {
        world_H_frame.setPosition(model->getCenterOfMassPosition());
        ok = model->getCenterOfMassJacobian(*m_jacobianCOM);
        int cols = m_jacobianCOM->cols();
        toEigen(*m_jacobian).block(0, 0, 3, cols) = toEigen(*m_jacobianCOM);
        toEigen(*m_jacobian).block(3, 0, 3, cols).setZero();
    }

    // Get the output signal memory location
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_FW_FRAME);
    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // Allocate objects for row-major -> col-major conversion
    Map<MatrixXdiDynTree> jacobianRowMajor = toEigen(*m_jacobian);
    Map<MatrixXdSimulink> jacobianColMajor(output.getBuffer<double>(), 6, 6 + dofs);

    // Forward the buffer to Simulink transforming it to ColMajor
    jacobianColMajor = jacobianRowMajor;
    return true;
}
