#include "SetReferences.h"

#include "Log.h"
#include "BlockInformation.h"
#include "Signal.h"
#include "RobotInterface.h"
#include <yarp/dev/ControlBoardInterfaces.h>

#define _USE_MATH_DEFINES
#include <cmath>

using namespace wbt;

const std::string SetReferences::ClassName = "SetReferences";

SetReferences::SetReferences()
: m_resetControlMode(true)
{}

void SetReferences::rad2deg(std::vector<double>& v)
{
    const double rad2deg = 180.0 / (2 * M_PI);
    for (auto& element : v) {
        element *= rad2deg;
    }
}

unsigned SetReferences::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 1;
}

bool SetReferences::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // Memory allocation / Saving data not allowed here

    if (!WBBlock::configureSizeAndPorts(blockInfo)) return false;

    const unsigned dofs = getConfiguration().getNumberOfDoFs();

    // INPUTS
    // ======
    //
    // 1) Joint refereces (1xDoFs vector)
    //

    // Number of inputs
    if (!blockInfo->setNumberOfInputPorts(1)) {
        Log::getSingleton().error("Failed to configure the number of input ports.");
        return false;
    }

    // Size and type
    bool success = blockInfo->setInputPortVectorSize(0, dofs);
    blockInfo->setInputPortType(0, PortDataTypeDouble);

    if (!success) {
        Log::getSingleton().error("Failed to configure input ports.");
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

bool SetReferences::initialize(const BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) return false;

    // Reading the control type
    std::string controlType;
    if (!blockInfo->getStringParameterAtIndex(WBBlock::numberOfParameters() + 1,
                                              controlType)) {
        Log::getSingleton().error("Could not read control type parameter.");
        return false;
    }

    // Initialize the std::vectors
    const unsigned dofs = getConfiguration().getNumberOfDoFs();
    m_controlModes.assign(dofs, VOCAB_CM_UNKNOWN);

    // IControlMode.h
    if (controlType == "Position") {
        m_controlModes.assign(dofs, VOCAB_CM_POSITION);
    }
    else if (controlType == "Position Direct") {
        m_controlModes.assign(dofs, VOCAB_CM_POSITION_DIRECT);
    }
    else if (controlType == "Velocity") {
        m_controlModes.assign(dofs, VOCAB_CM_VELOCITY);
    }
    else if (controlType == "Torque") {
        m_controlModes.assign(dofs, VOCAB_CM_TORQUE);
    }
    else if (controlType == "PWM") {
        m_controlModes.assign(dofs, VOCAB_CM_PWM);
    }
    else if (controlType == "Current") {
        m_controlModes.assign(dofs, VOCAB_CM_CURRENT);
    }
    else {
        Log::getSingleton().error("Control Mode not supported.");
        return false;
    }

    // Retain the ControlBoardRemapper
    if (!getRobotInterface()->retainRemoteControlBoardRemapper()) {
        Log::getSingleton().error("Failed to initialize the Robot Interface containing the Control Board Remapper.");
        return false;
    }

    m_resetControlMode = true;
    return true;
}

bool SetReferences::terminate(const BlockInformation* blockInfo)
{
    using namespace yarp::dev;
    bool ok = true;

    // Get the interface
    std::weak_ptr<IControlMode2> icmd2;
    ok = ok & getRobotInterface()->getInterface(icmd2);
    if (!ok) {
        Log::getSingleton().error("Failed to get the IControlMode2 interface.");
    }

    // Set  all the controlledJoints VOCAB_CM_POSITION
    const unsigned dofs = getConfiguration().getNumberOfDoFs();
    m_controlModes.assign(dofs, VOCAB_CM_POSITION);

    ok = ok & icmd2.lock()->setControlModes(m_controlModes.data());
    if (!ok) {
        Log::getSingleton().error("Failed to set control mode.");
    }

    // Release the RemoteControlBoardRemapper
    ok = ok & getRobotInterface()->releaseRemoteControlBoardRemapper();
    if (!ok) {
        Log::getSingleton().error("Failed to release the RemoteControlBoardRemapper.");
    }

    return ok && WBBlock::terminate(blockInfo);
}

bool SetReferences::initializeInitialConditions(const BlockInformation* /*blockInfo*/)
{
    // Simply reset the variable m_resetControlMode.
    // It will be read at the first cycle of output.
    m_resetControlMode = true;
    return true;
}

bool SetReferences::output(const BlockInformation* blockInfo)
{
    using namespace yarp::dev;

    // Set the control mode at the first run
    if (m_resetControlMode) {
        m_resetControlMode = false;
        // Get the interface
        std::weak_ptr<IControlMode2> icmd2;
        if (!getRobotInterface()->getInterface(icmd2)) {
            Log::getSingleton().error("Failed to get the IControlMode2 interface.");
            return false;
        }
        // Set the control mode to all the controlledJoints
        if (!icmd2.lock()->setControlModes(m_controlModes.data())) {
            Log::getSingleton().error("Failed to set control mode.");
            return false;
        }
    }

    // Get the signal
    Signal references = blockInfo->getInputPortSignal(0);
    unsigned signalWidth = blockInfo->getInputPortWidth(0);
    std::vector<double> referencesVector = references.getStdVector(signalWidth);

    bool ok = false;
    // TODO: here only the first element is checked
    switch (m_controlModes.front()) {
        case VOCAB_CM_UNKNOWN:
            Log::getSingleton().error("Control mode has not been successfully set.");
            return false;
            break;
        case VOCAB_CM_POSITION: {
            // Get the interface
            std::weak_ptr<IPositionControl> interface;
            if (!getRobotInterface()->getInterface(interface)) {
                Log::getSingleton().error("Failed to get IPositionControl interface.");
                return false;
            }
            // Convert from rad to deg
            rad2deg(referencesVector);
            // Set the references
            ok = interface.lock()->positionMove(referencesVector.data());
            break;
        }
        case VOCAB_CM_POSITION_DIRECT: {
            // Get the interface
            std::weak_ptr<IPositionDirect> interface;
            if (!getRobotInterface()->getInterface(interface)) {
                Log::getSingleton().error("Failed to get IPositionDirect interface.");
                return false;
            }
            // Convert from rad to deg
            rad2deg(referencesVector);
            // Set the references
            ok = interface.lock()->setPositions(referencesVector.data());
            break;
        }
        case VOCAB_CM_VELOCITY: {
            // Get the interface
            std::weak_ptr<IVelocityControl> interface;
            if (!getRobotInterface()->getInterface(interface)) {
                Log::getSingleton().error("Failed to get IVelocityControl interface.");
                return false;
            }
            // Convert from rad to deg
            rad2deg(referencesVector);
            // Set the references
            ok = interface.lock()->velocityMove(referencesVector.data());
            break;
        }
        case VOCAB_CM_TORQUE: {
            // Get the interface
            std::weak_ptr<ITorqueControl> interface;
            if (!getRobotInterface()->getInterface(interface)) {
                Log::getSingleton().error("Failed to get ITorqueControl interface.");
                return false;
            }
            ok = interface.lock()->setRefTorques(referencesVector.data());
            break;
        }
        case VOCAB_CM_PWM: {
            // Get the interface
            std::weak_ptr<IPWMControl> interface;
            if (!getRobotInterface()->getInterface(interface)) {
                Log::getSingleton().error("Failed to get IPWMControl interface.");
                return false;
            }
            ok = interface.lock()->setRefDutyCycles(referencesVector.data());
            break;
        }
        case VOCAB_CM_CURRENT: {
            // Get the interface
            std::weak_ptr<ICurrentControl> interface;
            if (!getRobotInterface()->getInterface(interface)) {
                Log::getSingleton().error("Failed to get ICurrentControl interface.");
                return false;
            }
            ok = interface.lock()->setRefCurrents(referencesVector.data());
            break;
        }
    }

    if (!ok) {
        Log::getSingleton().error("Failed to set references.");
        return false;
    }

    return true;
}
