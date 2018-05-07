/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "ModelPartitioner.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"
#include "RobotInterface.h"
#include "Signal.h"

#include <ostream>
#include <unordered_map>
#include <utility>
#include <vector>

using namespace wbt;

const std::string ModelPartitioner::ClassName = "ModelPartitioner";

const unsigned PARAM_IDX_BIAS = WBBlock::NumberOfParameters - 1;
const unsigned PARAM_IDX_DIRECTION = PARAM_IDX_BIAS + 1;

class ModelPartitioner::impl
{
public:
    bool vectorToControlBoards;
    std::shared_ptr<JointNameToYarpMap> jointNameToYarpMap;
    std::shared_ptr<JointNameToIndexInControlBoardMap> jointNameToIndexInControlBoardMap;
    std::shared_ptr<ControlBoardIndexLimit> controlBoardIndexLimit;
};

ModelPartitioner::ModelPartitioner()
    : pImpl{new impl()}
{}

unsigned ModelPartitioner::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool ModelPartitioner::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata directionMetadata(
        ParameterType::BOOL, PARAM_IDX_DIRECTION, 1, 1, "VectorToControlBoards");

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

    if (!ModelPartitioner::parseParameters(blockInfo)) {
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
    // Set them manually using the m_controlBoardIndexLimit map.
    //
    // Doing this now has the disadvantage of allocating the KinDynComputations and the
    // RemoteControlBoardRemapper already at this early stage, but this happens only at the
    // first execution of the model if the joint list doesn't change.
    //
    if (vectorToControlBoards) {
        pImpl->controlBoardIndexLimit = robotInterface->getControlBoardIdxLimit();
        if (!pImpl->controlBoardIndexLimit) {
            wbtError << "Failed to get the map CBIdx <--> CBMaxIdx.";
            return false;
        }
        for (const auto& cb : *pImpl->controlBoardIndexLimit) {
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

    if (!ModelPartitioner::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    if (!m_parameters.getParameter("VectorToControlBoards", pImpl->vectorToControlBoards)) {
        wbtError << "Failed to get input parameters.";
        return false;
    }

    // Get the RobotInterface
    const auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }

    pImpl->jointNameToYarpMap = robotInterface->getJointsMapString();
    pImpl->jointNameToIndexInControlBoardMap = robotInterface->getControlledJointsMapCB();

    if (!pImpl->jointNameToYarpMap) {
        wbtError << "Failed to get the joint map iDynTree <--> Yarp.";
        return false;
    }

    if (!pImpl->jointNameToIndexInControlBoardMap) {
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

    if (pImpl->vectorToControlBoards) {
        const Signal dofsSignal = blockInfo->getInputPortSignal(0);
        if (!dofsSignal.isValid()) {
            wbtError << "Failed to get the input signal buffer.";
            return false;
        }

        for (unsigned ithJoint = 0; ithJoint < configuration.getNumberOfDoFs(); ++ithJoint) {
            const std::string ithJointName = configuration.getControlledJoints()[ithJoint];
            // Get the ControlBoard number the ith joint belongs
            const ControlBoardIndex& controlBoardOfJoint =
                pImpl->jointNameToYarpMap->at(ithJointName).first;
            // Get the index of the ith joint inside the controlledJoints vector relative to
            // its ControlBoard
            const JointIndexInControlBoard jointIdxInCB =
                pImpl->jointNameToIndexInControlBoardMap->at(ithJointName);

            // Get the data to forward
            Signal ithOutput = blockInfo->getOutputPortSignal(controlBoardOfJoint);
            if (!ithOutput.set(jointIdxInCB, dofsSignal.get<double>(ithJoint))) {
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
            const ControlBoardIndex& controlBoardOfJoint =
                pImpl->jointNameToYarpMap->at(ithJointName).first;
            // Get the index of the ith joint inside the controlledJoints vector relative to
            // its ControlBoard
            const JointIndexInControlBoard jointIdxInCB =
                pImpl->jointNameToIndexInControlBoardMap->at(ithJointName);

            // Get the data to forward
            const Signal ithInput = blockInfo->getInputPortSignal(controlBoardOfJoint);
            if (!dofsSignal.set(ithJoint, ithInput.get<double>(jointIdxInCB))) {
                wbtError << "Failed to set the output signal.";
                return false;
            }
        }
    }
    return true;
}
