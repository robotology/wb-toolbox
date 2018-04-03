#include "WBBlock.h"

#include "AnyType.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"
#include "ToolboxSingleton.h"

#include "iDynTree/KinDynComputations.h"
#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <memory>
#include <string>

using namespace wbt;

const unsigned WBBlock::NumberOfParameters = Block::NumberOfParameters + 2;

const unsigned PARAM_IDX_BIAS = Block::NumberOfParameters - 1;
const unsigned ConfigurationParameterIndex = PARAM_IDX_BIAS + 1; // Struct from Simulink
const unsigned ConfBlockNameParameterIndex =
    PARAM_IDX_BIAS + 2; // Absolute name of the block containing the configuration

iDynTreeRobotState::iDynTreeRobotState(const unsigned& dofs, const std::array<double, 3>& gravity)
    : m_gravity(gravity.data(), 3)
    , m_jointsVelocity(dofs)
    , m_jointsPosition(dofs)
{
    m_jointsPosition.zero();
    m_jointsVelocity.zero();
}

bool WBBlock::setRobotState(const wbt::Signal* basePose,
                            const wbt::Signal* jointsPos,
                            const wbt::Signal* baseVelocity,
                            const wbt::Signal* jointsVelocity)
{
    // SAVE THE ROBOT STATE
    // ====================

    using namespace iDynTree;
    using namespace Eigen;
    typedef Matrix<double, 4, 4, ColMajor> Matrix4dSimulink;

    // Base pose
    // ---------

    if (basePose) {
        // Get the buffer
        double* buffer = basePose->getBuffer<double>();
        if (!buffer) {
            wbtError << "Failed to read the base pose from input port.";
            return false;
        }
        // Fill the data
        fromEigen(robotState.m_world_T_base, Matrix4dSimulink(buffer));
    }

    // Joints position
    // ---------------

    if (jointsPos) {
        // Get the buffer
        double* buffer = jointsPos->getBuffer<double>();
        if (!buffer) {
            wbtError << "Failed to read joints positions from input port.";
            return false;
        }
        // Fill the data
        for (auto i = 0; i < jointsPos->getWidth(); ++i) {
            robotState.m_jointsPosition.setVal(i, buffer[i]);
        }
    }

    // Base Velocity
    // -------------

    if (baseVelocity) {
        // Get the buffer
        double* buffer = baseVelocity->getBuffer<double>();
        if (!buffer) {
            wbtError << "Failed to read the base velocity from input port.";
            return false;
        }
        // Fill the data
        robotState.m_baseVelocity = Twist(LinVelocity(buffer, 3), AngVelocity(buffer + 3, 3));
    }

    // Joints velocity
    // ---------------

    if (jointsVelocity) {
        // Get the buffer
        double* buffer = jointsVelocity->getBuffer<double>();
        if (!buffer) {
            wbtError << "Failed to read joints velocities from input port.";
            return false;
        }
        // Fill the data
        for (auto i = 0; i < jointsVelocity->getWidth(); ++i) {
            robotState.m_jointsVelocity.setVal(i, buffer[i]);
        }
    }

    // UPDATE THE IDYNTREE ROBOT STATE WITH NEW DATA
    // =============================================

    const auto& model = getRobotInterface()->getKinDynComputations();

    if (!model) {
        wbtError << "Failed to access the KinDynComputations object.";
        return false;
    }

    bool ok = model->setRobotState(robotState.m_world_T_base,
                                   robotState.m_jointsPosition,
                                   robotState.m_baseVelocity,
                                   robotState.m_jointsVelocity,
                                   robotState.m_gravity);

    if (!ok) {
        wbtError << "Failed to set the iDynTree robot state.";
        return false;
    }

    return true;
}

unsigned WBBlock::numberOfParameters()
{
    return WBBlock::NumberOfParameters;
}

bool WBBlock::getWBToolboxParameters(Configuration& config, const BlockInformation* blockInfo)
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
    for (auto cell : cellCJ) {
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
    for (auto cell : cellCBN) {
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
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    std::vector<double> gravityVector;
    if (!s["GravityVector"]->asVectorDouble(gravityVector)) {
        Log::getSingleton().error("Cannot retrieve vector from GravityVector parameter.");
        return false;
    }
    std::array<double, 3> gravityArray;
    for (auto i = 0; i < 3; ++i) {
        gravityArray[i] = gravityVector[i];
    }

    // Create the ToolboxConfig object
    // ===============================

    // Populate a Property object that stores the input parameters
    config.setRobotName(robotName);
    config.setUrdfFile(urdfFile);
    config.setControlledJoints(controlledJoints);
    config.setControlBoardsNames(controlBoardsNames);
    config.setLocalName(localName);
    config.setGravityVector(gravityArray);

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

    // Initialize the configuration block with the unique identifier of the conf block name
    Configuration config(confKey);
    // Populate the configuration object with data from the Simulink's struct parameter
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
        wbtError << "Failed to configure WBBlock. Read 0 DoFs.";
        return false;
    }

    return true;
}

bool WBBlock::initialize(const BlockInformation* blockInfo)
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
        Log::getSingleton().error(
            "Failed to retrieve the configuration object from the singleton.");
        return false;
    }

    // Initialize the iDynTreeRobotState struct
    const unsigned& dofs = interface.numberOfDoFs(confKey);
    robotState = iDynTreeRobotState(dofs, getConfiguration().getGravityVector());

    return true;
}

bool WBBlock::terminate(const BlockInformation* /*blockInfo*/)
{
    return true;
}
