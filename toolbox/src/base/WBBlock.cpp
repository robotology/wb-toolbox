#include "WBBlock.h"

#include "BlockInformation.h"
#include "Log.h"
#include "ToolboxSingleton.h"
#include "Configuration.h"
#include "RobotInterface.h"
#include "AnyType.h"
#include <string>
#include <memory>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/Transform.h>

using namespace wbt;

const unsigned WBBlock::ConfigurationParameterIndex = 1; // Struct from Simulink
const unsigned WBBlock::ConfBlockNameParameterIndex = 2; // Absolute name of the block containing the
                                                         // configuration

iDynTreeRobotState::iDynTreeRobotState(const unsigned& dofs, const std::vector<double>& gravity)
: m_gravity(gravity.data(), 3)
, m_jointsVelocity(dofs)
, m_jointsPosition(dofs)
{
    m_jointsPosition.zero();
    m_jointsVelocity.zero();
}

unsigned WBBlock::numberOfParameters() { return 2; }

bool WBBlock::getWBToolboxParameters(Configuration& config, BlockInformation* blockInfo)
{
    // Infos
    // =====
    //
    // For what concern initialization, the callback order is:
    //
    // -> configureSizeAndPorts()
    //    -> initialize()
    //       -> initializeInitialConditions()
    //
    // Despite class objects after the configureSizeAndPorts() are destroyed and
    // reallocated before the initialize(), the ToolboxSingleton remains alive and
    // can store from this stage the configuration created by reading the parameters.
    //

    // Gather the parameters from the Mask of the Simulink Block
    // =========================================================

    // Get the struct containing the parameters
    AnyStruct s;
    if (!blockInfo->getStructAtIndex(ConfigurationParameterIndex, s)) {
        Log::getSingleton().error("Failed to retrieve the struct with parameters.");
        return false;
    }

    // Check the existence of all the fields
    try {
        s.at("RobotName");
        s.at("UrdfFile");
        s.at("ControlledJoints");
        s.at("ControlBoardsNames");
        s.at("LocalName");
        s.at("GravityVector");
    }
    catch (const std::out_of_range& e) {
        Log::getSingleton().error("Cannot retrieve one or more parameter from config struct.");
        return false;
    }

    // RobotName
    std::string robotName;
    if (!s["RobotName"]->asString(robotName)) {
        Log::getSingleton().error("Cannot retrieve string from RobotName parameter.");
        return false;
    }

    // UrdfFile
    std::string urdfFile;
    if (!s["UrdfFile"]->asString(urdfFile)) {
        Log::getSingleton().error("Cannot retrieve string from UrdfFile parameter.");
        return false;
    }

    // ControlledJoints
    AnyCell cellCJ;
    if (!s["ControlledJoints"]->asAnyCell(cellCJ)) {
        Log::getSingleton().error("Cannot retrieve ControlledJoints parameter.");
        return false;
    }

    std::vector<std::string> controlledJoints;
    for (auto cell: cellCJ) {
        std::string joint;
        if (!cell->asString(joint)) {
            Log::getSingleton().error("Failed to convert ControlledJoints from cell to strings.");
            return false;
        }
        controlledJoints.push_back(joint);
    }

    // ControlBoardsNames
    AnyCell cellCBN;
    if (!s["ControlBoardsNames"]->asAnyCell(cellCBN)) {
        Log::getSingleton().error("Cannot retrieve ControlBoardsNames parameter.");
        return false;
    }

    std::vector<std::string> controlBoardsNames;
    for (auto cell: cellCBN) {
        std::string joint;
        if (!cell->asString(joint)) {
            Log::getSingleton().error("Failed to convert ControlBoardsNames from cell to string.");
            return false;
        }
        controlBoardsNames.push_back(joint);
    }

    // LocalName
    std::string localName;
    if (!s["LocalName"]->asString(localName)) {
        Log::getSingleton().error("Cannot retrieve string from LocalName parameter.");
        return false;
    }

    std::vector<double> gravityVector;
    if (!s["GravityVector"]->asVectorDouble(gravityVector)) {
        Log::getSingleton().error("Cannot retrieve vector from GravityVector parameter.");
        return false;
    }

    // Create the ToolboxConfig object
    // ===============================

    // Populate a Property object that stores the input parameters
    config.setRobotName(robotName);
    config.setUrdfFile(urdfFile);
    config.setControlledJoints(controlledJoints);
    config.setControlBoardsNames(controlBoardsNames);
    config.setLocalName(localName);
    config.setGravityVector(gravityVector);

    return true;
}

const Configuration& WBBlock::getConfiguration()
{
    ToolboxSingleton& interface = ToolboxSingleton::sharedInstance();
    return interface.getConfiguration(confKey);
}

const std::shared_ptr<RobotInterface> WBBlock::getRobotInterface()
{
    ToolboxSingleton& interface = ToolboxSingleton::sharedInstance();
    return interface.getRobotInterface(confKey);
}

bool WBBlock::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // Remember not allocate / save any data in this function, it won't be persistent.

    // Setup the ToolboxSingleton
    // ==========================

    // Setup the DOFs (required by childs for configuring port sizes)
    // --------------------------------------------------------------

    // Initialize the ToolboxSingleton
    ToolboxSingleton& interface = ToolboxSingleton::sharedInstance();

    // Get the absolute name of the block from which the configuration has been retrieved.
    // This will be the key of the map containing all the configurations of the ToolboxSingleton.
    if (!blockInfo->getStringParameterAtIndex(ConfBlockNameParameterIndex, confKey)) {
        Log::getSingleton().error("Failed to retrieve the struct with parameters.");
        return false;
    }

    // Get the configuration struct from the block
    Configuration config;
    if (!getWBToolboxParameters(config, blockInfo)) {
        return false;
    }

    // Check if the configuration is valid
    if (!config.isValid()) {
        Log::getSingleton().error("The provided configuration is not valid.");
        return false;
    }

    // Store the configuration inside the ToolboxSingleton
    if (!interface.storeConfiguration(confKey, config)) {
        Log::getSingleton().error("Failed to configure ToolboxSingleton.");
        return false;
    }

    // Check if the DoFs are positive (and if the config has been stored successfully)
    const Configuration& configFromSingleton = getConfiguration();

    if (configFromSingleton.getNumberOfDoFs() < 1) {
        Log::getSingleton().error("Failed to configure WBBlock. Read 0 DoFs.");
        return false;
    }

    return true;
}

bool WBBlock::initialize(BlockInformation* blockInfo)
{
    // CONFIGURE the ToolboxSingleton
    // ==============================

    ToolboxSingleton& interface = ToolboxSingleton::sharedInstance();

    // Gather again confKey
    // Note: despite data for blocks is not persistent between configureSizeAndPorts()
    //       and initialize() (e.g. confKey), the singleton is not destroyed.
    //       This means that the stored configurations are still there.
    if (!blockInfo->getStringParameterAtIndex(ConfBlockNameParameterIndex, confKey)) {
        Log::getSingleton().error("Failed to retrieve the struct with parameters.");
        return false;
    }

    // Check if the key is valid
    if (!interface.isKeyValid(confKey)) {
        Log::getSingleton().error("Failed to retrieve the configuration object from the singleton.");
        return false;
    }

    // Initialize the iDynTreeRobotState struct
    const unsigned& dofs = interface.numberOfDoFs(confKey);
    robotState = iDynTreeRobotState(dofs, getConfiguration().getGravityVector());

    return true;
}

bool WBBlock::terminate(BlockInformation* /*blockInfo*/)
{
    return true;
}
