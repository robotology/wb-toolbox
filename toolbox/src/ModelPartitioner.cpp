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

bool ModelPartitioner::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)){
        return false;}

    // PARAMETERS
    // ==========
    //
    // 1) Boolean specifying if yarp2WBI (true) or WBI2yarp (false)
    //

    bool yarp2WBI = false;
    unsigned yarp2WBIParameterIdx = WBBlock::numberOfParameters() + 1;

    if (!blockInfo->getBooleanParameterAtIndex(yarp2WBIParameterIdx, yarp2WBI)) {
        wbtError << "Failed to get input parameters.";
        return false;
    }

    // INPUTS
    // ======
    //
    // yarp2WBI
    // --------
    //
    // 1) Vector containing the data vector (1 x DoFs)
    //
    // WBI2yarp
    // --------
    //
    // n signals) The n ControlBoards configured from the config block
    //

    bool ok;
    unsigned numberOfInputs;
    unsigned controlBoardsNumber = getConfiguration().getControlBoardsNames().size();

    if (yarp2WBI) {
        numberOfInputs = 1;
        ok = blockInfo->setNumberOfInputPorts(numberOfInputs);
        blockInfo->setInputPortVectorSize(0, getConfiguration().getNumberOfDoFs());
        blockInfo->setInputPortType(0, PortDataTypeDouble);
    }
    else {
        numberOfInputs = controlBoardsNumber;
        ok = blockInfo->setNumberOfInputPorts(numberOfInputs);
        // Set the size as dynamic
        for (unsigned i = 0; i < numberOfInputs; ++i) {
            blockInfo->setInputPortVectorSize(i, -1);
            blockInfo->setInputPortType(i, PortDataTypeDouble);
        }
    }

    if (!ok) {
        wbtError << "Failed to set input port number.";
        return false;
    }

    // OUTPUTS
    // =======
    //
    // yarp2WBI
    // --------
    //
    // n signals) The n ControlBoards configured from the config block
    //
    // WBI2yarp
    // --------
    //
    // 1) Vector containing the data vector (1 x DoFs)
    //

    unsigned numberOfOutputs;

    if (yarp2WBI) {
        numberOfOutputs = controlBoardsNumber;
        ok = blockInfo->setNumberOfOutputPorts(numberOfOutputs);
        // Set the size as dynamic
        for (unsigned i = 0; i < numberOfOutputs; ++i) {
            blockInfo->setOutputPortVectorSize(i, -1);
            blockInfo->setOutputPortType(i, PortDataTypeDouble);
        }
    }
    else {
        numberOfOutputs = 1;
        ok = blockInfo->setNumberOfOutputPorts(numberOfOutputs);
        blockInfo->setOutputPortVectorSize(0, getConfiguration().getNumberOfDoFs());
        blockInfo->setOutputPortType(0, PortDataTypeDouble);
    }

    // For some reason, the output ports widths in the yarp2WBI case are not detected
    // properly by Simulink if set as DYNAMICALLY_SIZED (-1).
    // Set them manually using the m_controlBoardIdxLimit map.
    //
    // Doing this now has the disadvantage of allocating the KinDynComputations and the
    // RemoteControlBoardRemapper already at this early stage, but this happens only at the
    // first execution of the model if the joint list doesn't change.
    //
    if (yarp2WBI) {
        m_controlBoardIdxLimit = getRobotInterface()->getControlBoardIdxLimit();
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

bool ModelPartitioner::initialize(const BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    unsigned yarp2WBIParameterIdx = WBBlock::numberOfParameters() + 1;
    if (!blockInfo->getBooleanParameterAtIndex(yarp2WBIParameterIdx, m_yarp2WBI)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    m_jointsMapString = getRobotInterface()->getJointsMapString();
    m_controlledJointsMapCB = getRobotInterface()->getControlledJointsMapCB();

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
    if (m_yarp2WBI) {
        Signal dofsSignal = blockInfo->getInputPortSignal(0);

        for (unsigned ithJoint = 0; ithJoint < getConfiguration().getNumberOfDoFs(); ++ithJoint) {
            const std::string ithJointName = getConfiguration().getControlledJoints()[ithJoint];
            // Get the ControlBoard number the ith joint belongs
            const cb_idx& controlBoardOfJoint = m_jointsMapString->at(ithJointName).first;
            // Get the index of the ith joint inside the controlledJoints vector relative to
            // its ControlBoard
            const controlledJointIdxCB contrJointIdxCB = m_controlledJointsMapCB->at(ithJointName);

            // Get the data to forward
            Signal ithOutput = blockInfo->getOutputPortSignal(controlBoardOfJoint);
            ithOutput.set(contrJointIdxCB, dofsSignal.get<double>(ithJoint));
        }
    }
    else {
        Signal dofsSignal = blockInfo->getOutputPortSignal(0);

        for (unsigned ithJoint = 0; ithJoint < getConfiguration().getNumberOfDoFs(); ++ithJoint) {
            const std::string ithJointName = getConfiguration().getControlledJoints()[ithJoint];
            // Get the ControlBoard number the ith joint belongs
            const cb_idx& controlBoardOfJoint = m_jointsMapString->at(ithJointName).first;
            // Get the index of the ith joint inside the controlledJoints vector relative to
            // its ControlBoard
            const controlledJointIdxCB contrJointIdxCB = m_controlledJointsMapCB->at(ithJointName);

            // Get the data to forward
            const Signal ithInput = blockInfo->getInputPortSignal(controlBoardOfJoint);
            dofsSignal.set(ithJoint, ithInput.get<double>(contrJointIdxCB));
        }
    }
    return true;
}
