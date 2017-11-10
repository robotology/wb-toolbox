#include "SetLowLevelPID.h"

#include "Log.h"
#include "RobotInterface.h"
#include "BlockInformation.h"
#include <yarp/dev/ControlBoardPid.h>
#include <algorithm>

namespace wbt {

    const std::string SetLowLevelPID::ClassName = "SetLowLevelPID";

    unsigned SetLowLevelPID::numberOfParameters()
    {
        return WBBlock::numberOfParameters()
        + 1  // WBTPIDConfig object
        + 1; // Control type
    }

    bool SetLowLevelPID::configureSizeAndPorts(BlockInformation* blockInfo)
    {
        if (!WBBlock::configureSizeAndPorts(blockInfo)) return false;

        // INPUTS
        // ======
        //
        // No inputs
        //

        if (!blockInfo->setNumberOfInputPorts(0)) {
            Log::getSingleton().error("Failed to configure the number of input ports.");
            return false;
        }

        // OUTPUTS
        // =======
        //
        // No outputs
        //

        if (!blockInfo->setNumberOfOutputPorts(0)) {
            Log::getSingleton().error("Failed to configure the number of output ports.");
            return false;
        }

        return true;
    }

    bool SetLowLevelPID::readWBTPidConfigObject(BlockInformation* blockInfo)
    {
        AnyStruct s;
        if (!blockInfo->getStructAtIndex(WBBlock::numberOfParameters() + 1, s)) {
            Log::getSingleton().error("Failed to retrieve the struct with parameters.");
            return false;
        }

        // Check the existence of all the fields
        try {
            s.at("P");
            s.at("I");
            s.at("D");
            s.at("jointList");
        }
        catch (const std::out_of_range& e) {
            Log::getSingleton().error("Cannot retrieve one or more parameter from parameter's struct.");
            return false;
        }

        // Proportional gains
        std::vector<double> Pvector;
        if (!s["P"]->asVectorDouble(Pvector)) {
            Log::getSingleton().error("Cannot retrieve vector from P parameter.");
            return false;
        }

        // Integral gains
        std::vector<double> Ivector;
        if (!s["I"]->asVectorDouble(Ivector)) {
            Log::getSingleton().error("Cannot retrieve vector from I parameter.");
            return false;
        }

        // Derivative gains
        std::vector<double> Dvector;
        if (!s["D"]->asVectorDouble(Dvector)) {
            Log::getSingleton().error("Cannot retrieve vector from D parameter.");
            return false;
        }

        // Considered joint names
        AnyCell jointPidsCell;
        if (!s["jointList"]->asAnyCell(jointPidsCell)) {
            Log::getSingleton().error("Cannot retrieve string from jointList parameter.");
            return false;
        }

        // From AnyCell to vector<string>
        std::vector<std::string> jointNamesFromParameters;
        for (auto cell: jointPidsCell) {
            std::string joint;
            if (!cell->asString(joint)) {
                Log::getSingleton().error("Failed to convert jointList from cell to strings.");
                return false;
            }
            jointNamesFromParameters.push_back(joint);
        }

        assert(Pvector.size() == Ivector.size() == Dvector.size() == jointNamesFromParameters.size());

        // Store this data into a private member map
        for (unsigned i = 0; i < jointNamesFromParameters.size(); ++i) {
            // Check the processed joint is actually a controlledJoint
            const auto& controlledJoints = getConfiguration().getControlledJoints();
            auto findElement = std::find(std::begin(controlledJoints),
                                         std::end(controlledJoints),
                                         jointNamesFromParameters[i]);
            if (findElement != std::end(controlledJoints)) {
                m_pidJointsFromParameters[jointNamesFromParameters[i]] = {Pvector[i], Ivector[i], Dvector[i]};
            }
            else {
                Log::getSingleton().warning("Attempted to set PID of joint " + jointNamesFromParameters[i] +
                                            " non currently controlled. Skipping it.");
            }

        }

        if (m_pidJointsFromParameters.size() != jointNamesFromParameters.size()) {
            Log::getSingleton().warning("PID have been passed only for a subset of the controlled joints.");
        }

        return true;
    }

    bool SetLowLevelPID::initialize(BlockInformation* blockInfo)
    {
        if (!WBBlock::initialize(blockInfo)) return false;

        // Reading the control type
        std::string controlType;
        if (!blockInfo->getStringParameterAtIndex(WBBlock::numberOfParameters() + 2, controlType)) {
            Log::getSingleton().error("Could not read control type parameter.");
            return false;
        }

        if (controlType == "Position") {
            m_controlType = yarp::dev::VOCAB_PIDTYPE_POSITION;
        }
        else if (controlType == "Torque") {
            m_controlType = yarp::dev::VOCAB_PIDTYPE_TORQUE;
        }
        else {
            Log::getSingleton().error("Control type not recognized.");
            return false;
        }

        // Reading the WBTPIDConfig matlab class
        if (!readWBTPidConfigObject(blockInfo)) {
            Log::getSingleton().error("Failed to parse the WBTPIDConfig object.");
            return 1;
        }

        // Retain the RemoteControlBoardRemapper
        if (!getRobotInterface()->retainRemoteControlBoardRemapper()) {
            Log::getSingleton().error("Failed to retain the control board remapper.");
            return false;
        }

        const unsigned& dofs = getConfiguration().getNumberOfDoFs();

        // Initialize the vector size to the number of dofs
        m_defaultPidValues.resize(dofs);

        // Get the interface
        std::weak_ptr<yarp::dev::IPidControl> iPidControl;
        if (!getRobotInterface()->getInterface(iPidControl)) {
            Log::getSingleton().error("Failed to get IPidControl interface.");
            return false;
        }

        // Store the default gains
        if (!iPidControl.lock()->getPids(m_controlType, m_defaultPidValues.data())) {
            Log::getSingleton().error("Failed to get default data from IPidControl.");
            return false;
        }

        // Initialize the vector of the applied pid gains with the default gains
        m_appliedPidValues = m_defaultPidValues;

        // Override the PID with the gains specified as block parameters
        for (unsigned i = 0; i < dofs; ++i) {
            const std::string jointName = getConfiguration().getControlledJoints()[i];
            // If the pid has been passed, set the new gains
            if (m_pidJointsFromParameters.find(jointName) != m_pidJointsFromParameters.end()) {
                PidData gains = m_pidJointsFromParameters[jointName];
                m_appliedPidValues[i].setKp(std::get<PGAIN>(gains));
                m_appliedPidValues[i].setKi(std::get<IGAIN>(gains));
                m_appliedPidValues[i].setKd(std::get<DGAIN>(gains));
            }
        }

        // Apply the new pid gains
        if (!iPidControl.lock()->setPids(m_controlType, m_appliedPidValues.data())) {
            Log::getSingleton().error("Failed to set PID values.");
            return false;
        }

        return true;
    }

    bool SetLowLevelPID::terminate(BlockInformation* blockInfo)
    {
        bool ok = true;

        // Get the IPidControl interface
        std::weak_ptr<yarp::dev::IPidControl> iPidControl;
        ok = ok & getRobotInterface()->getInterface(iPidControl);
        if (!ok) {
            Log::getSingleton().error("Failed to get IPidControl interface.");
        }

        // Reset default pid gains
        ok = ok & iPidControl.lock()->setPids(m_controlType, m_defaultPidValues.data());
        if (!ok) {
            Log::getSingleton().error("Failed to set default PID values.");
        }

        // Release the RemoteControlBoardRemapper
        ok = ok & getRobotInterface()->releaseRemoteControlBoardRemapper();
        if (!ok) {
            Log::getSingleton().error("Failed to release the RemoteControlBoardRemapper.");
        }

        return ok && WBBlock::terminate(blockInfo);
    }

    bool SetLowLevelPID::output(BlockInformation* blockInfo)
    {
        return true;
    }
}
