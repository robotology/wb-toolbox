/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "RobotInterface.h"
#include "Configuration.h"
#include "Log.h"

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/FreeFloatingMatrices.h>
#include <iDynTree/Model/Indices.h>
#include <iDynTree/Model/Model.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/os/ResourceFinder.h>

#include <cassert>
#include <chrono>
#include <functional>
#include <sstream>
#include <thread>
#include <utility>
#include <vector>

using namespace wbt;

// ====================
// ROBOTINTERFACE PIMPL
// ====================

struct YarpInterfaces
{
    yarp::dev::IControlMode* iControlMode = nullptr;
    yarp::dev::IPositionControl* iPositionControl = nullptr;
    yarp::dev::IPositionDirect* iPositionDirect = nullptr;
    yarp::dev::IVelocityControl* iVelocityControl = nullptr;
    yarp::dev::ITorqueControl* iTorqueControl = nullptr;
    yarp::dev::IPWMControl* iPWMControl = nullptr;
    yarp::dev::ICurrentControl* iCurrentControl = nullptr;
    yarp::dev::IEncoders* iEncoders = nullptr;
    yarp::dev::IMotorEncoders* iMotorEncoders = nullptr;
    yarp::dev::IControlLimits* iControlLimits = nullptr;
    yarp::dev::IPidControl* iPidControl = nullptr;
};

class RobotInterface::impl
{
public:
    std::unique_ptr<yarp::dev::PolyDriver> robotDevice;
    std::shared_ptr<iDynTree::KinDynComputations> kinDynComp;
    YarpInterfaces yarpInterfaces;

    // Maps used to store infos about yarp's and idyntree's internal joint indexing
    std::shared_ptr<JointIndexToYarpMap> jointIndexToYarpMap;
    std::shared_ptr<JointNameToYarpMap> jointNameToYarpMap;
    std::shared_ptr<JointNameToIndexInControlBoardMap> jointNameToIndexInControlBoardMap;
    std::shared_ptr<ControlBoardIndexLimit> controlBoardIndexLimit;

    const wbt::Configuration config; // Configuration from Simulink Block's parameters
    wbt::RobotInterface* robotInterface = nullptr; // Pointer to the RobotInterface object

    impl() = delete;
    explicit impl(const Configuration& configuration)
        : config(configuration)
    {}

    template <typename T>
    T* getInterfaceLazyEval(T*& interface,
                            const std::unique_ptr<yarp::dev::PolyDriver>& cbRemapper);

    bool checkInterface(std::function<bool(double*)>& getMeasurement)
    {
        // This method is used only on interfaces which get measurements. There is an interval right
        // after the allocation of the RemoteControlBoardRemapper when a get*() calls from a viewed
        // interface will return false. This failure is not due to a wrong usage of the interface,
        // but rather to a call before the interface actually receives data.
        unsigned counter = 0;
        constexpr unsigned maxIter = 100;
        std::vector<double> buffer(config.getNumberOfDoFs(), 0);

        while (!getMeasurement(buffer.data())) {
            if (++counter == maxIter) {
                wbtError << "Failed to get a measurement during the interface initialization.";
                return false;
            }
            // Sleep for some while
            std::this_thread::sleep_for(std::chrono::microseconds(500));
        }
        return true;
    }

    void setOwner(wbt::RobotInterface* r) { robotInterface = r; }

    bool initializeModel()
    {
        assert(!kinDynComp);

        // Allocate the object
        kinDynComp = std::make_shared<iDynTree::KinDynComputations>();
        if (!kinDynComp)
            return false;

        // Explicitly set the velocity representation
        kinDynComp->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

        // Use RF to load the urdf file
        // ----------------------------

        // Initialize RF
        // Workaround for the fact that ResourceFinder initializes the network by itself. See
        // YARP#1014
        using namespace yarp::os;
        Network network;
        ResourceFinder& rf = ResourceFinder::getResourceFinderSingleton();
        rf.configure(0, nullptr);

        // Get the absolute path of the urdf file
        std::string urdf_file = config.getUrdfFile();
        std::string urdf_file_path = rf.findFile(urdf_file.c_str());

        // Fail if the file is not found
        if (urdf_file_path.empty()) {
            wbtError << "ResourceFinder couldn't find urdf file " + urdf_file + ".";
            return false;
        }

        // Load the reduced model into KinDynComputations
        // ----------------------------------------------

        // Load the joint list
        std::vector<std::string> controlledJoints = config.getControlledJoints();

        // Use ModelLoader to load the reduced model
        iDynTree::ModelLoader mdlLoader;
        if (!mdlLoader.loadReducedModelFromFile(urdf_file_path, controlledJoints)) {
            wbtError << "Impossible to load " + urdf_file + "." << std::endl
                     << "Possible causes: file not found, or the joint "
                     << "list contains an entry not present in the urdf model.";
            return false;
        }

        // Add the loaded model to the KinDynComputations object
        return kinDynComp->loadRobotModel(mdlLoader.model());
    }

    bool initializeRemoteControlBoardRemapper()
    {
        // Initialize the network
        yarp::os::Network::init();
        if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
            wbtError << "YARP server wasn't found active.";
            return false;
        }

        // Object where the RemoteControlBoardRemapper options will be stored
        yarp::os::Property options;

        // Name of the device
        options.put("device", "remotecontrolboardremapper");

        // Controlled joints (axes)
        yarp::os::Bottle axesNames;
        yarp::os::Bottle& axesList = axesNames.addList();
        for (auto axis : config.getControlledJoints()) {
            axesList.addString(axis);
        }
        options.put("axesNames", axesNames.get(0));

        // ControlBoard names
        yarp::os::Bottle remoteControlBoards;
        yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
        for (auto cb : config.getControlBoardsNames()) {
            remoteControlBoardsList.addString("/" + config.getRobotName() + "/" + cb);
        }
        options.put("remoteControlBoards", remoteControlBoards.get(0));

        // Prefix of the openened ports
        // In this case appending the unique id is necessary, since multiple configuration can
        // share some ControlBoard in their RemoteControlBoardRemappers. In this case, it is not
        // possible using the same prefix for all the RemoteControlBoardRemapper devices.
        options.put("localPortPrefix", config.getLocalName() + "/" + config.getUniqueId());

        // Misc options
        yarp::os::Property& remoteCBOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
        remoteCBOpts.put("writeStrict", "on");

        // Allocate the interface driver
        robotDevice = std::unique_ptr<yarp::dev::PolyDriver>(new yarp::dev::PolyDriver());

        if (!robotDevice) {
            wbtError << "Failed to instantiante an empty PolyDriver class.";
            return false;
        }

        // Open the interface driver
        if (!robotDevice->open(options) && !robotDevice->isValid()) {
            // Remove garbage if the opening fails
            robotDevice.reset();
            wbtError << "Failed to open the RemoteControlBoardRemapper with the options passed.";
            return false;
        }

        return true;
    }

    /**
     * @brief Map joints between iDynTree and Yarp indices
     *
     * Creates the map between joints (specified as either names or idyntree indices) and
     * their YARP representation, which consist in a pair: Control Board index and joint index
     * inside the its Control Board.
     *
     * @see RobotInterface::getJointsMapString, RobotInterface::getJointsMapIndex
     *
     * @return True if the map has been created successfully, false otherwise.
     */
    bool mapDoFs()
    {
        // Initialize the network
        yarp::os::Network::init();
        if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
            wbtError << "YARP server wasn't found active.";
            return false;
        }

        std::vector<std::unique_ptr<yarp::dev::IAxisInfo>> iAxisInfos;

        for (unsigned cbNum = 0; cbNum < config.getControlBoardsNames().size(); ++cbNum) {

            std::unique_ptr<yarp::dev::PolyDriver> controlBoard;
            const std::string prefix = "/" + config.getRobotName() + "/";
            const std::string remoteName = prefix + config.getControlBoardsNames().at(cbNum);

            if (!getSingleControlBoard(remoteName, controlBoard)) {
                return false;
            }

            // Get an IAxisInfo object from the interface
            std::unique_ptr<yarp::dev::IAxisInfo> iAxisInfo;
            yarp::dev::IAxisInfo* iAxisInfoPtr = iAxisInfo.get();
            controlBoard->view(iAxisInfoPtr);
            if (!iAxisInfoPtr) {
                wbtError << "Unable to open IAxisInfo from " << remoteName << ".";
                return false;
            }

            // Get an IEncoders object from the interface
            // This is used to get how many joints the control board contains
            std::unique_ptr<yarp::dev::IEncoders> iEnc;
            yarp::dev::IEncoders* iEncPtr = iEnc.get();
            controlBoard->view(iEncPtr);
            int numAxes;
            if (!iEncPtr || !iEncPtr->getAxes(&numAxes)) {
                wbtError << "Unable to open IEncoders from " << remoteName << ".";
                return false;
            }

            int found = -1;
            // Iterate all the joints in the selected Control Board
            for (unsigned axis = 0; axis < numAxes; ++axis) {
                std::string axisName;
                if (!iAxisInfoPtr->getAxisName(axis, axisName)) {
                    wbtError << "Unable to get AxisName from " << remoteName << ".";
                    return false;
                }
                // Look if axisName is a controlledJoint
                for (const auto& controlledJoint : config.getControlledJoints()) {
                    if (controlledJoint == axisName) {
                        found++;
                        // Get the iDynTree index from the model
                        const auto& kinDynComp = robotInterface->getKinDynComputations();
                        if (!kinDynComp) {
                            wbtError << "Failed to get KinDynComputations.";
                            return false;
                        }
                        const auto& model = kinDynComp->model();
                        iDynTree::JointIndex iDynJointIdx = model.getJointIndex(axisName);
                        if (iDynJointIdx == iDynTree::JOINT_INVALID_INDEX) {
                            wbtError << "Joint " << axisName << " exists in the " << remoteName
                                     << " control board but not in the model.";
                            return false;
                        }
                        // If this is the first entry to add, allocate the objects
                        if (!jointIndexToYarpMap) {
                            jointIndexToYarpMap = std::make_shared<JointIndexToYarpMap>();
                        }
                        if (!jointNameToYarpMap) {
                            jointNameToYarpMap = std::make_shared<JointNameToYarpMap>();
                        }
                        if (!jointNameToIndexInControlBoardMap) {
                            jointNameToIndexInControlBoardMap =
                                std::make_shared<JointNameToIndexInControlBoardMap>();
                        }
                        if (!controlBoardIndexLimit) {
                            controlBoardIndexLimit = std::make_shared<ControlBoardIndexLimit>();
                        }
                        // Create a new entry in the map objects
                        jointNameToYarpMap->insert(
                            std::make_pair(controlledJoint, std::make_pair(cbNum, axis)));
                        jointIndexToYarpMap->insert(std::make_pair(static_cast<int>(iDynJointIdx),
                                                                   std::make_pair(cbNum, axis)));
                        jointNameToIndexInControlBoardMap->insert(
                            std::make_pair(controlledJoint, found));
                        (*controlBoardIndexLimit)[cbNum] = found + 1;
                        break;
                    }
                }
            }

            // Notify that the control board just checked is not used by any joint
            // of the controlledJoints list
            if (found < 0) {
                wbtWarning << "No controlled joints found in "
                                  + config.getControlBoardsNames().at(cbNum)
                                  + " control board. It might be unused.";
            }

            // Close the ControlBoard device
            if (!controlBoard->close()) {
                wbtError << "Unable to close the interface of the Control Board.";
                return false;
            }
        }

        // Initialize the network
        yarp::os::Network::fini();

        return true;
    }

    /**
     * @brief Create a RemoteControlBoard object for a given remoteName
     *
     * @see mapDoFs
     *
     * @param remoteName Name of the remote from which the remote control board is be initialized
     * @param[out] controlBoard Smart pointer to the allocated remote control board
     * @return True if success, false otherwise.
     */
    bool getSingleControlBoard(const std::string& remoteName,
                               std::unique_ptr<yarp::dev::PolyDriver>& controlBoard)
    {
        // Configure the single control board
        yarp::os::Property options;
        options.clear();
        options.put("device", "remote_controlboard");
        options.put("remote", remoteName);
        options.put("local", config.getLocalName() + "/CBtmp");
        options.put("writeStrict", "on");

        // Initialize the device
        controlBoard = std::unique_ptr<yarp::dev::PolyDriver>(new yarp::dev::PolyDriver());
        if (!controlBoard) {
            wbtError << "Failed to retain the RemoteControlBoard "
                     << "used for mapping iDynTree - YARP DoFs.";
            return false;
        }

        // Try to open the control board
        if (!controlBoard->open(options) || !controlBoard->isValid()) {
            wbtError << "Unable to open RemoteControlBoard " << remoteName << ".";
            return false;
        }

        return true;
    }
};

// ==============
// ROBOTINTERFACE
// ==============

// CONSTRUCTOR / DESTRUCTOR
// ========================

RobotInterface::RobotInterface(const wbt::Configuration& config)
    : pImpl{new impl(config)}
{
    pImpl->setOwner(this);
}

RobotInterface::~RobotInterface()
{
    // Close the RemoteControlBoardRemapper
    if (pImpl->robotDevice) {
        pImpl->robotDevice->close();
    }
    // Finalize the network
    yarp::os::Network::fini();
}

// GET METHODS
// ===========

const wbt::Configuration& RobotInterface::getConfiguration() const
{
    return pImpl->config;
}

const std::shared_ptr<JointNameToYarpMap> RobotInterface::getJointsMapString()
{
    if (!pImpl->jointNameToYarpMap || pImpl->jointNameToYarpMap->empty()) {
        if (!pImpl->mapDoFs()) {
            wbtError << "Failed to create the joint maps.";
            return nullptr;
        }
    }

    return pImpl->jointNameToYarpMap;
}

const std::shared_ptr<JointIndexToYarpMap> RobotInterface::getJointsMapIndex()
{
    if (!pImpl->jointIndexToYarpMap || pImpl->jointIndexToYarpMap->empty()) {
        if (!pImpl->mapDoFs()) {
            wbtError << "Failed to create the joint maps.";
            return nullptr;
        }
    }

    assert(pImpl->jointIndexToYarpMap);
    return pImpl->jointIndexToYarpMap;
}

const std::shared_ptr<JointNameToIndexInControlBoardMap> RobotInterface::getControlledJointsMapCB()
{
    if (!pImpl->jointNameToIndexInControlBoardMap
        || pImpl->jointNameToIndexInControlBoardMap->empty()) {
        if (!pImpl->mapDoFs()) {
            wbtError << "Failed to create joint maps.";
            return nullptr;
        }
    }

    assert(pImpl->jointNameToIndexInControlBoardMap);
    return pImpl->jointNameToIndexInControlBoardMap;
}

const std::shared_ptr<ControlBoardIndexLimit> RobotInterface::getControlBoardIdxLimit()
{
    if (!pImpl->controlBoardIndexLimit || pImpl->controlBoardIndexLimit->empty()) {
        if (!pImpl->mapDoFs()) {
            wbtError << "Failed to create joint maps.";
            return nullptr;
        }
    }

    assert(pImpl->controlBoardIndexLimit);
    return pImpl->controlBoardIndexLimit;
}

const std::shared_ptr<iDynTree::KinDynComputations> RobotInterface::getKinDynComputations()
{
    if (pImpl->kinDynComp) {
        return pImpl->kinDynComp;
    }

    // Otherwise, initialize a new object
    if (!pImpl->initializeModel()) {
        wbtError << "Failed to initialize the KinDynComputations object.";
        // Return an empty shared_ptr (implicitly initialized)
        return nullptr;
    }

    return pImpl->kinDynComp;
}

// TEMPLATED METHODS
// =================

template <typename T>
T* RobotInterface::impl::getInterfaceLazyEval(
    T*& interface,
    const std::unique_ptr<yarp::dev::PolyDriver>& cbRemapper)
{
    if (!interface) {
        // Lazy-initialize the RemoteControlBoardRemapper device
        if (!cbRemapper) {
            if (!initializeRemoteControlBoardRemapper()) {
                wbtError << "Failed to initialize the RemoteControlBoardRemapper.";
                return nullptr;
            }
        }
        // Ask the interface from the device
        if (!robotDevice->view(interface)) {
            wbtError << "Failed to view the interface.";
            return nullptr;
        }
    }

    // Return the raw pointer
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IControlMode*& interface)
{
    interface = pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iControlMode, pImpl->robotDevice);
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(yarp::dev::IPositionControl*& interface)
{
    interface =
        pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iPositionControl, pImpl->robotDevice);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IPositionDirect*& interface)
{
    interface =
        pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iPositionDirect, pImpl->robotDevice);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IVelocityControl*& interface)
{
    interface =
        pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iVelocityControl, pImpl->robotDevice);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::ITorqueControl*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iTorqueControl;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }
        // Check if it works fine
        std::function<bool(double*)> getMeas = std::bind(
            &yarp::dev::ITorqueControl::getTorques, storedInterface, std::placeholders::_1);
        if (!pImpl->checkInterface(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(yarp::dev::IPWMControl*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iPWMControl;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }
        // Check if it works fine
        std::function<bool(double*)> getMeas = std::bind(
            &yarp::dev::IPWMControl::getDutyCycles, storedInterface, std::placeholders::_1);
        if (!pImpl->checkInterface(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(yarp::dev::ICurrentControl*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iCurrentControl;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }
        // Check if it works fine
        std::function<bool(double*)> getMeas = std::bind(
            &yarp::dev::ICurrentControl::getCurrents, storedInterface, std::placeholders::_1);
        if (!pImpl->checkInterface(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(yarp::dev::IEncoders*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iEncoders;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }
        // Check if it works fine
        std::function<bool(double*)> getMeas =
            std::bind(&yarp::dev::IEncoders::getEncoders, storedInterface, std::placeholders::_1);
        if (!pImpl->checkInterface(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(yarp::dev::IMotorEncoders*& interface)
{
    auto& storedInterface = pImpl->yarpInterfaces.iMotorEncoders;

    if (!storedInterface) {
        // Get the interface
        if (!pImpl->getInterfaceLazyEval(storedInterface, pImpl->robotDevice)) {
            return false;
        }
        // Check if it works fine
        std::function<bool(double*)> getMeas = std::bind(
            &yarp::dev::IMotorEncoders::getMotorEncoders, storedInterface, std::placeholders::_1);
        if (!pImpl->checkInterface(getMeas)) {
            return false;
        }
    }

    // Return a pointer to the interface to the caller
    interface = storedInterface;
    return static_cast<bool>(interface);
}

template <>
bool RobotInterface::getInterface(yarp::dev::IControlLimits*& interface)
{
    interface =
        pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iControlLimits, pImpl->robotDevice);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IPidControl*& interface)
{
    interface = pImpl->getInterfaceLazyEval(pImpl->yarpInterfaces.iPidControl, pImpl->robotDevice);
    return interface;
}
