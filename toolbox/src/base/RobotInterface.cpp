#include "RobotInterface.h"
#include "Log.h"
#include <sstream>
#include <iterator>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/os/Property.h>

namespace wbt {

    // The declaration of the following template specializations are required only by GCC
    using namespace yarp::dev;
    template <> bool RobotInterface::getInterface(std::weak_ptr<IControlMode2>& interface);
    template <> bool RobotInterface::getInterface(std::weak_ptr<IPositionControl>& interface);
    template <> bool RobotInterface::getInterface(std::weak_ptr<IPositionDirect>& interface);
    template <> bool RobotInterface::getInterface(std::weak_ptr<IVelocityControl>& interface);
    template <> bool RobotInterface::getInterface(std::weak_ptr<ITorqueControl>& interface);
    template <> bool RobotInterface::getInterface(std::weak_ptr<IPWMControl>& interface);
    template <> bool RobotInterface::getInterface(std::weak_ptr<ICurrentControl>& interface);
    template <> bool RobotInterface::getInterface(std::weak_ptr<IEncoders>& interface);
    template <> bool RobotInterface::getInterface(std::weak_ptr<IControlLimits2>& interface);
    template <> bool RobotInterface::getInterface(std::weak_ptr<IPidControl>& interface);
}

using namespace wbt;

// CONSTRUCTOR / DESTRUCTOR
// ========================


RobotInterface::RobotInterface(const wbt::Configuration& c)
: m_config(c)
, m_robotDeviceCounter(0)
{}

RobotInterface::~RobotInterface()
{
    // Asserts for debugging purpose.

    // - 1 if at least one block asked the model. At this point only the shared_ptr
    //     of m_kinDynComp should be still alive (--> 1)
    // - 0 if no block asked for the model. m_kinDynComp was never allocated.
    assert(m_kinDynComp.use_count() <= 1);

    // m_robotDevice should be destroyed by the last releaseCB()
    assert(!m_robotDevice);
    assert(m_robotDeviceCounter == 0);
}

bool RobotInterface::mapDoFs()
{
    std::unique_ptr<yarp::dev::PolyDriver> controlBoard;
    std::vector<std::unique_ptr<yarp::dev::IAxisInfo>> iAxisInfos;
    yarp::os::Property options;

    for (unsigned cbNum = 0; cbNum < m_config.getControlBoardsNames().size(); ++cbNum) {
        // Configure the single control board
        options.clear();
        options.put("device","remotecontrolboard");
        const std::string prefix = "/" + m_config.getRobotName() + "/";
        const std::string remoteName = prefix + m_config.getControlBoardsNames().at(cbNum);
        options.put("remote", remoteName);
        options.put("localPortPrefix", "WBTtmp");

        // Try to open the control board
        if (!controlBoard->open(options) || !controlBoard->isValid()) {
            Log::getSingleton().error("Unable to open RemoteControlBoard " + remoteName);
            return false;
        }

        // Get an IAxisInfo object from the device
        std::unique_ptr<yarp::dev::IAxisInfo> iAxisInfo;
        yarp::dev::IAxisInfo* iAxisInfoPtr = iAxisInfo.get();
        controlBoard->view(iAxisInfoPtr);
        if (!iAxisInfoPtr) {
            Log::getSingleton().error("Unable to open IAxisInfo from " + remoteName);
            return false;
        }

        // Get an IEncoders object from the device
        // This is used to get how many joints the control board contains
        std::unique_ptr<yarp::dev::IEncoders> iEnc;
        yarp::dev::IEncoders* iEncPtr = iEnc.get();
        controlBoard->view(iEncPtr);
        int numAxes;
        if (!iEncPtr || !iEncPtr->getAxes(&numAxes)) {
            Log::getSingleton().error("Unable to open IEncoders from " + remoteName);
            return false;
        }

        // Iterate all the joints in the selected Control Board
        for (unsigned axis = 0; axis < numAxes; ++axis) {
            std::string axisName;
            if (!iAxisInfoPtr->getAxisName(axis, axisName)) {
                Log::getSingleton().error("Unable to get AxisName from " + remoteName);
                return false;
            }
            // Look if axisName is a controlledJoint
            bool found = false;
            for (const auto& controlledJoint : m_config.getControlledJoints()) {
                if (controlledJoint == axisName) {
                    // Get the iDynTree index
                    const auto& model = getKinDynComputations()->model();
                    iDynTree::LinkIndex iDynLinkIdx = model.getLinkIndex(axisName);
                    if (iDynLinkIdx == iDynTree::LINK_INVALID_INDEX) {
                        Log::getSingleton().error("Joint " + axisName + " exists in the " +
                                                  remoteName + "control board but not in the model.");
                        return false;
                    }
                    // If this is the first entry to add, allocate the objects
                    if (!m_jointsMapIndex) {
                        m_jointsMapIndex = std::make_shared<JointsMapIndex>();
                    }
                    if (!m_jointsMapString) {
                        m_jointsMapString = std::make_shared<JointsMapString>();
                    }
                    // Create a new entry in the map objects
                    m_jointsMapString->at(controlledJoint) = {cbNum, axis};
                    m_jointsMapIndex->at(static_cast<int>(iDynLinkIdx)) = {cbNum, axis};
                    found = true;
                    break;
                }
            }
            // Notify that the control board just checked is not used by any joint
            // of the controlledJoints list
            if (!found) {
                Log::getSingleton().warning("No controlled joints found in " +
                                            m_config.getControlBoardsNames().at(cbNum) +
                                            " control board. It might be unsed.");
            }
        }
    }

    if (!controlBoard->close()) {
        Log::getSingleton().error("Unable to close the device of the Control Board.");
        return false;
    }
    return true;
}

// GET METHODS
// ===========

const wbt::Configuration& RobotInterface::getConfiguration() const
{
    return m_config;
}

const std::shared_ptr<JointsMapString> RobotInterface::getJointsMapString()
{
    if (m_jointsMapString->empty()) {
        if (!mapDoFs()) {
            Log::getSingleton().error("Failed to create the Yarp - iDynTree joint map.");
            return nullptr;
        }
    }

    assert (m_jointsMapString);
    return m_jointsMapString;
}

const std::shared_ptr<JointsMapIndex> RobotInterface::getJointsMapIndex()
{
    if (m_jointsMapIndex->empty()) {
        if (!mapDoFs()) {
            Log::getSingleton().error("Failed to create the Yarp - iDynTree joint map.");
            return nullptr;
        }
    }

    assert (m_jointsMapIndex);
    return m_jointsMapIndex;
}

const std::shared_ptr<iDynTree::KinDynComputations> RobotInterface::getKinDynComputations()
{
    if (m_kinDynComp) {
        return m_kinDynComp;
    }

    // Otherwise, initialize a new object
    if (initializeModel()) {
        return m_kinDynComp;
    }

    // Return an empty shared_ptr (implicitly initialized)
    return nullptr;
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::IControlMode2>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iControlMode2);
    return !interface.expired();
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::IPositionControl>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iPositionControl);
    return !interface.expired();
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::IPositionDirect>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iPositionDirect);
    return !interface.expired();
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::IVelocityControl>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iVelocityControl);
    return !interface.expired();
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::ITorqueControl>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iTorqueControl);
    return !interface.expired();
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::IPWMControl>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iPWMControl);
    return !interface.expired();
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::ICurrentControl>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iCurrentControl);
    return !interface.expired();
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::IEncoders>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iEncoders);
    return !interface.expired();
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::IControlLimits2>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iControlLimits2);
    return !interface.expired();
}

template <>
bool RobotInterface::getInterface(std::weak_ptr<yarp::dev::IPidControl>& interface)
{
    interface = getInterfaceFromTemplate(m_yarpDevices.iPidControl);
    return !interface.expired();
}

// LAZY EVALUATION
// ===============

bool RobotInterface::retainRemoteControlBoardRemapper()
{
    if (m_robotDeviceCounter > 0) {
        m_robotDeviceCounter++;
        return true;
    }

    assert(!m_robotDevice);
    if (m_robotDevice) {
        m_robotDevice.reset();
    }

    if (!initializeRemoteControlBoardRemapper()) {
        return false;
    }

    m_robotDeviceCounter++;
    return true;
}

bool RobotInterface::releaseRemoteControlBoardRemapper()
{
    // The RemoteControlBoardRemapper has not been used
    if (m_robotDeviceCounter == 0) {
        return true;
    }

    // If there are at most 2 blocks with th CB still used
    if (m_robotDeviceCounter > 1) {
        m_robotDeviceCounter--;
        return true;
    }

    // This should be executed by the last block which uses CB (m_robotDeviceCounter=1)
    assert(m_robotDevice);
    if (m_robotDevice) {
        // Free all the drivers
        m_yarpDevices.iControlMode2.reset();
        m_yarpDevices.iPositionControl.reset();
        m_yarpDevices.iPositionDirect.reset();
        m_yarpDevices.iVelocityControl.reset();
        m_yarpDevices.iTorqueControl.reset();
        m_yarpDevices.iPWMControl.reset();
        m_yarpDevices.iCurrentControl.reset();
        m_yarpDevices.iEncoders.reset();
        m_yarpDevices.iControlLimits2.reset();
        m_yarpDevices.iPidControl.reset();
        //  Close the device
        m_robotDevice->close();
        // Free the object
        m_robotDevice.reset();
    }

    // Initialize the network
    yarp::os::Network::init();

    m_robotDeviceCounter = 0;
    return true;
}

// INITIALIZATION HELPERS
// ======================

bool RobotInterface::initializeModel()
{
    assert (!m_kinDynComp);

    // Allocate the object
    m_kinDynComp = std::make_shared<iDynTree::KinDynComputations>();
    if (!m_kinDynComp) return false;

    // Use RF to load the urdf file
    // ----------------------------

    // Initialize RF
    // Workaround for the fact that ResourceFinder initializes the network by itself. See YARP#1014
    using namespace yarp::os;
    Network network;
    ResourceFinder& rf = ResourceFinder::getResourceFinderSingleton();
    rf.configure(0, 0);

    // Get the absolute path of the urdf file
    std::string urdf_file = getConfiguration().getUrdfFile();
    std::string urdf_file_path = rf.findFile(urdf_file.c_str());

    // Load the reduced model into KinDynComputations
    // ----------------------------------------------

    // Load the joint list
    std::vector<std::string> controlledJoints = getConfiguration().getControlledJoints();

    // Use ModelLoader to load the reduced model
    iDynTree::ModelLoader mdlLoader;
    if (!mdlLoader.loadReducedModelFromFile(urdf_file_path, controlledJoints)) {
        Log::getSingleton().error("ToolboxSingleton: impossible to load " + urdf_file + ".");
        Log::getSingleton().error("Probably the joint list contains an entry not present in the urdf model.");
        return false;
    }

    // Add the loaded model to the KinDynComputations object
    return m_kinDynComp->loadRobotModel(mdlLoader.model());
}

bool RobotInterface::initializeRemoteControlBoardRemapper()
{
    // Initialize the network
    yarp::os::Network::init();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        Log::getSingleton().error("YARP server wasn't found active!!");
        return false;
    }

    // Object where the RemoteControlBoardRemapper options will be stored
    yarp::os::Property options;

    // Name of the device
    options.put("device", "remotecontrolboardremapper");

    // Controlled joints (axes)
    yarp::os::Bottle axesNames;
    yarp::os::Bottle& axesList = axesNames.addList();
    for (auto axis : m_config.getControlledJoints()) {
        axesList.addString(axis);
    }
    options.put("axesNames",axesNames.get(0));

    // ControlBoard names
    yarp::os::Bottle remoteControlBoards;
    yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
    for (auto cb : m_config.getControlBoardsNames()) {
        remoteControlBoardsList.addString("/" + m_config.getRobotName() + "/" + cb);
    }
    options.put("remoteControlBoards", remoteControlBoards.get(0));

    // Prefix of the openened ports
    // In this case appending the unique id is necessary, since multiple configuration can
    // share some ControlBoard in their RemoteControlBoardRemappers. In this case, it is not
    // possible using the same prefix for all the RemoteControlBoardRemapper devices.
    options.put("localPortPrefix", m_config.getLocalName() + "/" + m_config.getUniqueId());

    // Misc options
    yarp::os::Property& remoteCBOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteCBOpts.put("writeStrict", "on");

    // If resources have been properly cleaned, there should be no allocated device.
    // However, if blocks fail and they don't terminate, the state of the singleton
    // could be not clean.
    if (m_robotDevice) {
        // Force the release
        m_robotDeviceCounter = 1;
        Log::getSingleton().warning("The ToolboxSingleton state is dirty. Trying to clean the state before proceeding.");
        if (!releaseRemoteControlBoardRemapper()) {
            Log::getSingleton().error("Failed to force the release of the RemoteControlBoardRemapper. ");
            return false;
        }
    }

    // Allocate the interface driver
    m_robotDevice = std::unique_ptr<yarp::dev::PolyDriver>(new yarp::dev::PolyDriver());

    if (!m_robotDevice) {
        Log::getSingleton().error("Failed to instantiante an empty PolyDriver class.");
        return false;
    }

    // Open the interface driver
    if (!m_robotDevice->open(options) && !m_robotDevice->isValid()) {
        // Remove garbage if the opening fails
        m_robotDevice.reset();
        Log::getSingleton().error("Failed to open the RemoteControlBoardRemapper with the options passed.");
        return false;
    }

    return true;
}

// OTHER METHODS
// =============

template <typename T>
std::weak_ptr<T> RobotInterface::getInterfaceFromTemplate(std::shared_ptr<T> device)
{
    if (!device) {
        // Blocks which require the RemoteControlBoardRemapper need to retain / release it
        // in their initialization / terminate phase;
        assert(m_robotDevice);
        if (!m_robotDevice) {
            // Return an empty weak pointer
            return std::weak_ptr<T>();
        }

        T* ptr = nullptr;
        if (!m_robotDevice->view(ptr)) {
            // Return an empty weak_ptr
            return std::weak_ptr<T>();
        }
        // Store ptr into the smart pointer
        device.reset(ptr);
    }
    // Implicit conversion from shared_ptr to weak_ptr
    return device;
}
