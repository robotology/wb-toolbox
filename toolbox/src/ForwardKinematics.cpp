#include "ForwardKinematics.h"

#include "BlockInformation.h"
#include "Signal.h"
#include "Log.h"
#include "RobotInterface.h"
#include <memory>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <Eigen/Core>

using namespace wbt;

const std::string ForwardKinematics::ClassName = "ForwardKinematics";

const unsigned ForwardKinematics::INPUT_IDX_BASE_POSE = 0;
const unsigned ForwardKinematics::INPUT_IDX_JOINTCONF = 1;
const unsigned ForwardKinematics::OUTPUT_IDX_FW_FRAME = 0;

ForwardKinematics::ForwardKinematics()
: m_frameIsCoM(false)
, m_frameIndex(iDynTree::FRAME_INVALID_INDEX)
{}

unsigned ForwardKinematics::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool ForwardKinematics::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // Memory allocation / Saving data not allowed here

    if (!WBBlock::configureSizeAndPorts(blockInfo)) return false;

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
    // 1) Homogeneous transformation between the world and the specified frame (4x4 matrix)
    //

    // Number of outputs
    if (!blockInfo->setNumberOfOutputPorts(1)) {
        Log::getSingleton().error("Failed to configure the number of output ports.");
        return false;
    }

    // Size and type
    success = blockInfo->setOutputPortMatrixSize(OUTPUT_IDX_FW_FRAME, 4, 4);
    blockInfo->setOutputPortType(OUTPUT_IDX_FW_FRAME, PortDataTypeDouble);

    return success;
}

bool ForwardKinematics::initialize(const BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) return false;

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
        Log::getSingleton().error("Cannot retrieve handle to WBI model.");
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

    return true;
}

bool ForwardKinematics::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool ForwardKinematics::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    typedef Matrix<double, 4, 4, ColMajor> Matrix4dSimulink;
    typedef Matrix<double, 4, 4, Eigen::RowMajor> Matrix4diDynTree;

    const auto& model = getRobotInterface()->getKinDynComputations();

    if (!model) {
        Log::getSingleton().error("Failed to retrieve the KinDynComputations object.");
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    Signal basePoseSig = blockInfo->getInputPortSignal(INPUT_IDX_BASE_POSE);
    Signal jointsPosSig = blockInfo->getInputPortSignal(INPUT_IDX_JOINTCONF);

    bool ok = setRobotState(&basePoseSig,
                            &jointsPosSig,
                            nullptr,
                            nullptr);

    if (!ok) {
        Log::getSingleton().error("Failed to set the robot state.");
        return false;
    }

    // OUTPUT
    // ======

    iDynTree::Transform world_H_frame;

    // Compute the transform to the selected frame
    if (!m_frameIsCoM) {
        world_H_frame = model->getWorldTransform(m_frameIndex);
    }
    else {
        world_H_frame.setPosition(model->getCenterOfMassPosition());
    }

    // Get the output signal memory location
    Signal output = blockInfo->getOutputPortSignal(OUTPUT_IDX_FW_FRAME);

    // Allocate objects for row-major -> col-major conversion
    Map<const Matrix4diDynTree> world_H_frame_RowMajor = toEigen(world_H_frame.asHomogeneousTransform());
    Map<Matrix4dSimulink> world_H_frame_ColMajor(output.getBuffer<double>(),
                           4, 4);

    // Forward the buffer to Simulink transforming it to ColMajor
    world_H_frame_ColMajor = world_H_frame_RowMajor;
    return true;
}
