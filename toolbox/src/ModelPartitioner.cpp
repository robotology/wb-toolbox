#include "ModelPartitioner.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <algorithm>

using namespace wbt;

const std::string ModelPartitioner::ClassName = "ModelPartitioner";

const unsigned PARAM_IDX_BIAS = WBBlock::NumberOfParameters - 1;
const unsigned PARAM_IDX_DIRECTION = PARAM_IDX_BIAS + 1;

unsigned ModelPartitioner::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool ModelPartitioner::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata directionMetadata(
        PARAM_BOOL, PARAM_IDX_DIRECTION, 1, 1, "VectorToControlBoards");

    bool ok = blockInfo->addParameterMetadata(directionMetadata);

    if (!ok) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool ModelPartitioner::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    if (!parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    // PARAMETERS
    // ==========
    //
    // 1) Boolean specifying if VectorToControlBoards (true) or ControlBoardsToVector (false)
    //

    bool vectorToControlBoards;
    if (!m_parameters.getParameter("VectorToControlBoards", vectorToControlBoards)) {
        wbtError << "Failed to get input parameters.";
        return false;
    }

    // INPUTS
    // ======
    //
    // VectorToControlBoards
    // ---------------------
    //
    // 1) Vector containing the data vector (1 x DoFs)
    //
    // ControlBoardsToVector
    // ---------------------
    //
    // n signals) The n ControlBoards configured from the config block
    //

    // Get the number of the control boards
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }
    const auto dofs = robotInterface->getConfiguration().getNumberOfDoFs();
    const auto controlBoardsNumber =
        robotInterface->getConfiguration().getControlBoardsNames().size();

    bool ok;
    unsigned numberOfInputs;

    if (vectorToControlBoards) {
        numberOfInputs = 1;
        ok = blockInfo->setNumberOfInputPorts(numberOfInputs);
        blockInfo->setInputPortVectorSize(0, dofs);
        blockInfo->setInputPortType(0, DataType::DOUBLE);
    }
    else {
        numberOfInputs = controlBoardsNumber;
        ok = blockInfo->setNumberOfInputPorts(numberOfInputs);
        // Set the sizes as dynamic, they will be filled in the initialize() method
        for (unsigned i = 0; i < numberOfInputs; ++i) {
            blockInfo->setInputPortVectorSize(i, Signal::DynamicSize);
            blockInfo->setInputPortType(i, DataType::DOUBLE);
        }
    }

    if (!ok) {
        wbtError << "Failed to set input port number.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // VectorToControlBoards
    // ---------------------
    //
    // n signals) The n ControlBoards configured from the config block
    //
    // ControlBoardsToVector
    // ---------------------
    //
    // 1) Vector containing the data vector (1 x DoFs)
    //

    unsigned numberOfOutputs;

    if (vectorToControlBoards) {
        numberOfOutputs = controlBoardsNumber;
        ok = blockInfo->setNumberOfOutputPorts(numberOfOutputs);
        // Set the sizes as dynamic, they will be filled in the initialize() method
        for (unsigned i = 0; i < numberOfOutputs; ++i) {
            blockInfo->setOutputPortVectorSize(i, Signal::DynamicSize);
            blockInfo->setOutputPortType(i, DataType::DOUBLE);
        }
    }
    else {
        numberOfOutputs = 1;
        ok = blockInfo->setNumberOfOutputPorts(numberOfOutputs);
        blockInfo->setOutputPortVectorSize(0, dofs);
        blockInfo->setOutputPortType(0, DataType::DOUBLE);
    }

    // For some reason, the output ports widths in the yarp2WBI case are not detected
    // properly by Simulink if set as DYNAMICALLY_SIZED (-1).
    // Set them manually using the m_controlBoardIdxLimit map.
    //
    // Doing this now has the disadvantage of allocating the KinDynComputations and the
    // RemoteControlBoardRemapper already at this early stage, but this happens only at the
    // first execution of the model if the joint list doesn't change.
    //
    if (vectorToControlBoards) {
        m_controlBoardIdxLimit = robotInterface->getControlBoardIdxLimit();
        if (!m_controlBoardIdxLimit) {
            wbtError << "Failed to get the map CBIdx <--> CBMaxIdx.";
            return false;
        }
        for (const auto& cb : *m_controlBoardIdxLimit) {
            if (!blockInfo->setOutputPortVectorSize(cb.first, cb.second)) {
                wbtError << "Failed to set ouput port size reading them from cb map.";
                return false;
            }
        }
    }

    if (!ok) {
        wbtError << "Failed to set output port number.";
        return false;
    }

    return true;
}

bool ModelPartitioner::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    if (!parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    if (!m_parameters.getParameter("VectorToControlBoards", m_vectorToControlBoards)) {
        wbtError << "Failed to get input parameters.";
        return false;
    }

    // Get the RobotInterface
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }

    m_jointsMapString = robotInterface->getJointsMapString();
    m_controlledJointsMapCB = robotInterface->getControlledJointsMapCB();

    if (!m_jointsMapString) {
        wbtError << "Failed to get the joint map iDynTree <--> Yarp.";
        return false;
    }

    if (!m_controlledJointsMapCB) {
        wbtError << "Failed to get the joint map iDynTree <--> controlledJointsIdx.";
        return false;
    }

    return true;
}

bool ModelPartitioner::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool ModelPartitioner::output(const BlockInformation* blockInfo)
{
    // Get the RobotInterface
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }

    // Get the Configuration
    const auto& configuration = robotInterface->getConfiguration();

    if (m_vectorToControlBoards) {
        const Signal dofsSignal = blockInfo->getInputPortSignal(0);
        if (!dofsSignal.isValid()) {
            wbtError << "Failed to get the input signal buffer.";
            return false;
        }

        for (unsigned ithJoint = 0; ithJoint < configuration.getNumberOfDoFs(); ++ithJoint) {
            const std::string ithJointName = configuration.getControlledJoints()[ithJoint];
            // Get the ControlBoard number the ith joint belongs
            const cb_idx& controlBoardOfJoint = m_jointsMapString->at(ithJointName).first;
            // Get the index of the ith joint inside the controlledJoints vector relative to
            // its ControlBoard
            const controlledJointIdxCB contrJointIdxCB = m_controlledJointsMapCB->at(ithJointName);

            // Get the data to forward
            Signal ithOutput = blockInfo->getOutputPortSignal(controlBoardOfJoint);
            if (!ithOutput.set(contrJointIdxCB, dofsSignal.get<double>(ithJoint))) {
                wbtError << "Failed to set the output signal.";
                return false;
            }
        }
    }
    else {
        Signal dofsSignal = blockInfo->getOutputPortSignal(0);
        if (!dofsSignal.isValid()) {
            wbtError << "Failed to get the input signal buffer.";
            return false;
        }

        for (unsigned ithJoint = 0; ithJoint < configuration.getNumberOfDoFs(); ++ithJoint) {
            const std::string ithJointName = configuration.getControlledJoints()[ithJoint];
            // Get the ControlBoard number the ith joint belongs
            const cb_idx& controlBoardOfJoint = m_jointsMapString->at(ithJointName).first;
            // Get the index of the ith joint inside the controlledJoints vector relative to
            // its ControlBoard
            const controlledJointIdxCB contrJointIdxCB = m_controlledJointsMapCB->at(ithJointName);

            // Get the data to forward
            const Signal ithInput = blockInfo->getInputPortSignal(controlBoardOfJoint);
            if (!dofsSignal.set(ithJoint, ithInput.get<double>(contrJointIdxCB))) {
                wbtError << "Failed to set the output signal.";
                return false;
            }
        }
    }
    return true;
}
