#include "ModelPartitioner.h"

#include "BlockInformation.h"
#include "Configuration.h"
#include "RobotInterface.h"
#include "Signal.h"
#include "Log.h"

namespace wbt {

    const std::string ModelPartitioner::ClassName = "ModelPartitioner";

    unsigned ModelPartitioner::numberOfParameters()
    {
        return 1;
    }

    bool ModelPartitioner::configureSizeAndPorts(BlockInformation* blockInfo)
    {
        if (!WBBlock::configureSizeAndPorts(blockInfo)) return false;

        // PARAMETERS
        // ==========
        //
        // 1) Boolean specifying if yarp2WBI (true) or WBI2yarp (false)
        //

        bool yarp2WBI;
        if (!blockInfo->getBooleanParameterAtIndex(1, yarp2WBI)) {
            Log::getSingleton().error("Failed to get input parameters.");
            return false;
        }

        // TODO: update to new logic
        // INPUTS
        // ======
        //
        // yarp2WBI
        // --------
        //
        // 1) Map of joints / control boards (1 x nJoints)
        // 2) Vector containing the full set of robot's joints (1 x nJoints)
        //
        // WBI2yarp
        // --------
        //
        // 1) Map of joints / control boards (1 x nJoints)
        // 2) Torso     Control Board Signals
        // 3) Head      Control Board Signals
        // 4) Left  Arm Control Board Signals
        // 5) Right Arm Control Board Signals
        // 6) Left  Leg Control Board Signals
        // 7) Right Leg Control Board Signals
        //

        bool ok;
        unsigned numberOfInputs;
        unsigned controlBoardsNumber = getConfiguration().getControlBoardsNames().size();

        if (yarp2WBI) {
            numberOfInputs = 1;
            ok = blockInfo->setNumberOfInputPorts(numberOfInputs);
            blockInfo->setInputPortVectorSize(0, getConfiguration().getNumberOfDoFs());
        }
        else {
            numberOfInputs = controlBoardsNumber;
            ok = blockInfo->setNumberOfInputPorts(numberOfInputs);
            // Set the size as dynamic
            for (unsigned i = 0; i < numberOfInputs; ++i) {
                blockInfo->setInputPortType(i, PortDataTypeDouble);
            }
        }

        if (!ok) {
            Log::getSingleton().error("Failed to set input port number.");
            return false;
        }


        // TODO: update
        // OUTPUTS
        // =======
        //
        // yarp2WBI
        // --------
        //
        // 1) Torso     Control Board Signals
        // 2) Head      Control Board Signals
        // 3) Left  Arm Control Board Signals
        // 4) Right Arm Control Board Signals
        // 5) Left  Leg Control Board Signals
        // 6) Right Leg Control Board Signals
        //
        // WBI2yarp
        // --------
        //
        // 1) Vector containing the full set of robot's joints (1 x nJoints)
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

        if (!ok) {
            Log::getSingleton().error("Failed to set output port number.");
            return false;
        }

        return true;
    }

    bool ModelPartitioner::initialize(BlockInformation* blockInfo)
    {
        if (!WBBlock::initialize(blockInfo)) return false;

        if (!blockInfo->getBooleanParameterAtIndex(1, m_yarp2WBI)) {
            Log::getSingleton().error("Failed to get input parameters.");
            return false;
        }

        m_jointsMapString = getRobotInterface()->getJointsMapString();

        if (!m_jointsMapString) {
            return false;
        }

        return true;
    }

    bool ModelPartitioner::terminate(BlockInformation* blockInfo)
    {
        return WBBlock::terminate(blockInfo);
    }

    bool ModelPartitioner::output(BlockInformation* blockInfo)
    {
        if (m_yarp2WBI) {
            // Input
            Signal jointListSignal = blockInfo->getInputPortSignal(0);

            // Outputs
            Signal ithControlBoardSignal;

            for (unsigned ithJoint = 0; ithJoint < blockInfo->getInputPortWidth(0); ++ithJoint) {
                // Get the ControlBoard number the ith joint belongs, and its index into the CB itself
                std::string ithJointName = getConfiguration().getControlledJoints()[ithJoint];
                const auto& mappedJointInfos = m_jointsMapString->at(ithJointName);
                cb_idx controlBoardOfJoint = mappedJointInfos.first;
                jointIdx_yarp yarpIndexOfJoint = mappedJointInfos.second;

                // Get the data to forward
                Data value = jointListSignal.get(ithJoint);

                // Forward the value to the correct output / output index
                ithControlBoardSignal = blockInfo->getOutputPortSignal(controlBoardOfJoint);
                ithControlBoardSignal.set(yarpIndexOfJoint, value.doubleData());
            }
        }
        else {
            // Inputs
            Signal ithControlBoardSignal;

            // Output
            Signal jointListSignal = blockInfo->getOutputPortSignal(0);

            for (unsigned ithJoint = 0; ithJoint < blockInfo->getOutputPortWidth(0); ++ithJoint) {
                // Get the ControlBoard number the ith joint belongs, and its index into the CB itself
                std::string ithJointName = getConfiguration().getControlledJoints()[ithJoint];
                const auto& mappedJointInfos = m_jointsMapString->at(ithJointName);
                cb_idx controlBoardOfJoint = mappedJointInfos.first;
                jointIdx_yarp yarpIndexOfJoint = mappedJointInfos.second;

                // Get the data to forward
                ithControlBoardSignal = blockInfo->getInputPortSignal(controlBoardOfJoint);
                Data value = ithControlBoardSignal.get(ithJoint);

                // Forward the value to the correct output index
                jointListSignal.set(yarpIndexOfJoint, value.doubleData());
            }
        }
        return true;
    }
}
