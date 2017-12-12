#include "RobotInterface.h"
#include "Log.h"
#include <sstream>
#include <iterator>
#include <utility>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/os/Property.h>
#include <yarp/os/Bottle.h>

namespace wbt {
    // The declaration of the following template specializations are required only by GCC
    using namespace yarp::dev;
    template <> bool RobotInterface::getInterface(IControlMode2*& interface);
    template <> bool RobotInterface::getInterface(IPositionControl*& interface);
    template <> bool RobotInterface::getInterface(IPositionDirect*& interface);
    template <> bool RobotInterface::getInterface(IVelocityControl*& interface);
    template <> bool RobotInterface::getInterface(ITorqueControl*& interface);
    template <> bool RobotInterface::getInterface(IPWMControl*& interface);
    template <> bool RobotInterface::getInterface(ICurrentControl*& interface);
    template <> bool RobotInterface::getInterface(IEncoders*& interface);
    template <> bool RobotInterface::getInterface(IControlLimits2*& interface);
    template <> bool RobotInterface::getInterface(IPidControl*& interface);
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

bool RobotInterface::getSingleControlBoard(const std::string& remoteName, std::unique_ptr<yarp::dev::PolyDriver>& controlBoard) {
    // Configure the single control board
    yarp::os::Property options;
    options.clear();
    options.put("device", "remote_controlboard");
    options.put("remote", remoteName);
    options.put("local", m_config.getLocalName() + "/CBtmp");
    options.put("writeStrict", "on");

    // Initialize the device
    controlBoard = std::unique_ptr<yarp::dev::PolyDriver>(new yarp::dev::PolyDriver());
    if (!controlBoard) {
        Log::getSingleton().error("Failed to retain the RemoteControlBoard ");
        Log::getSingleton().errorAppend("used for mapping iDynTree - YARP DoFs.");
        return false;
    }

    // Try to open the control board
    if (!controlBoard->open(options) || !controlBoard->isValid()) {
        Log::getSingleton().error("Unable to open RemoteControlBoard " + remoteName);
        return false;
    }

    return true;
}

bool RobotInterface::mapDoFs()
{
    // Initialize the network
    yarp::os::Network::init();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        Log::getSingleton().error("YARP server wasn't found active!!");
        return false;
    }

    std::vector<std::unique_ptr<yarp::dev::IAxisInfo>> iAxisInfos;

    for (unsigned cbNum = 0; cbNum < m_config.getControlBoardsNames().size(); ++cbNum) {

        std::unique_ptr<yarp::dev::PolyDriver> controlBoard;
        const std::string prefix = "/" + m_config.getRobotName() + "/";
        const std::string remoteName = prefix + m_config.getControlBoardsNames().at(cbNum);

        if (!getSingleControlBoard(remoteName, controlBoard)) {
            return false;
        }

        // Get an IAxisInfo object from the interface
        std::unique_ptr<yarp::dev::IAxisInfo> iAxisInfo;
        yarp::dev::IAxisInfo* iAxisInfoPtr = iAxisInfo.get();
        controlBoard->view(iAxisInfoPtr);
        if (!iAxisInfoPtr) {
            Log::getSingleton().error("Unable to open IAxisInfo from " + remoteName);
            return false;
        }

        // Get an IEncoders object from the interface
        // This is used to get how many joints the control board contains
        std::unique_ptr<yarp::dev::IEncoders> iEnc;
        yarp::dev::IEncoders* iEncPtr = iEnc.get();
        controlBoard->view(iEncPtr);
        int numAxes;
        if (!iEncPtr || !iEncPtr->getAxes(&numAxes)) {
            Log::getSingleton().error("Unable to open IEncoders from " + remoteName);
            return false;
        }

        int found = -1;
        // Iterate all the joints in the selected Control Board
        for (unsigned axis = 0; axis < numAxes; ++axis) {
            std::string axisName;
            if (!iAxisInfoPtr->getAxisName(axis, axisName)) {
                Log::getSingleton().error("Unable to get AxisName from " + remoteName);
                return false;
            }
            // Look if axisName is a controlledJoint
            for (const auto& controlledJoint : m_config.getControlledJoints()) {
                if (controlledJoint == axisName) {
                    found++;
                    // Get the iDynTree index from the model
                    const auto& kinDynComp = getKinDynComputations();
                    if (!kinDynComp) {
                        Log::getSingleton().error("Failed to get KinDynComputations.");
                        return false;
                    }
                    const auto& model = kinDynComp->model();
                    iDynTree::JointIndex iDynJointIdx = model.getJointIndex(axisName);
                    if (iDynJointIdx == iDynTree::JOINT_INVALID_INDEX) {
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
                    if (!m_controlledJointsMapCB) {
                        m_controlledJointsMapCB = std::make_shared<ControlledJointsMapCB>();
                    }
                    if (!m_controlBoardIdxLimit) {
                        m_controlBoardIdxLimit = std::make_shared<ControlBoardIdxLimit>();
                    }
                    // Create a new entry in the map objects
                    m_jointsMapString->insert(std::make_pair(controlledJoint, std::make_pair(cbNum, axis)));
                    m_jointsMapIndex->insert(std::make_pair(static_cast<int>(iDynJointIdx),
                                                            std::make_pair(cbNum, axis)));
                    m_controlledJointsMapCB->insert(std::make_pair(controlledJoint, found));
                    (*m_controlBoardIdxLimit)[cbNum] = found + 1;
                    break;
                }
            }

        }

        // Notify that the control board just checked is not used by any joint
        // of the controlledJoints list
        if (found < 0) {
            Log::getSingleton().warning("No controlled joints found in " +
            m_config.getControlBoardsNames().at(cbNum) +
            " control board. It might be unused.");
        }

        // Close the ControlBoard device
        if (!controlBoard->close()) {
            Log::getSingleton().error("Unable to close the interface of the Control Board.");
            return false;
        }
    }

    // Initialize the network
    yarp::os::Network::fini();

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
    if (!m_jointsMapString || m_jointsMapString->empty()) {
        if (!mapDoFs()) {
            Log::getSingleton().error("Failed to create the joint maps.");
            return nullptr;
        }
    }

    return m_jointsMapString;
}

const std::shared_ptr<JointsMapIndex> RobotInterface::getJointsMapIndex()
{
    if (!m_jointsMapIndex || m_jointsMapIndex->empty()) {
        if (!mapDoFs()) {
            Log::getSingleton().error("Failed to create the joint maps.");
            return nullptr;
        }
    }

    assert (m_jointsMapIndex);
    return m_jointsMapIndex;
}

const std::shared_ptr<ControlledJointsMapCB> RobotInterface::getControlledJointsMapCB()
{
    if (!m_controlledJointsMapCB || m_controlledJointsMapCB->empty()) {
        if (!mapDoFs()) {
            Log::getSingleton().error("Failed to create joint maps.");
            return nullptr;
        }
    }

    assert (m_controlledJointsMapCB);
    return m_controlledJointsMapCB;
}

const std::shared_ptr<ControlBoardIdxLimit> RobotInterface::getControlBoardIdxLimit()
{
    if (!m_controlBoardIdxLimit || m_controlBoardIdxLimit->empty()) {
        if (!mapDoFs()) {
            Log::getSingleton().error("Failed to create joint maps.");
            return nullptr;
        }
    }

    assert (m_controlBoardIdxLimit);
    return m_controlBoardIdxLimit;
}

const std::shared_ptr<iDynTree::KinDynComputations> RobotInterface::getKinDynComputations()
{
    if (m_kinDynComp) {
        return m_kinDynComp;
    }

    // Otherwise, initialize a new object
    if (!initializeModel()) {
        Log::getSingleton().error("Failed to initialize the KinDynComputations object.");
        // Return an empty shared_ptr (implicitly initialized)
        return nullptr;
    }

    return m_kinDynComp;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IControlMode2*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iControlMode2);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IPositionControl*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iPositionControl);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IPositionDirect*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iPositionDirect);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IVelocityControl*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iVelocityControl);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::ITorqueControl*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iTorqueControl);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IPWMControl*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iPWMControl);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::ICurrentControl*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iCurrentControl);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IEncoders*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iEncoders);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IControlLimits2*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iControlLimits2);
    return interface;
}

template <>
bool RobotInterface::getInterface(yarp::dev::IPidControl*& interface)
{
    interface = getInterfaceFromTemplate(m_yarpInterfaces.iPidControl);
    return interface;
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
        Log::getSingleton().error("First initialization of the RemoteControlBoardRemapper failed.");
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
        // Set to zero all the pointers to the interfaces
        m_yarpInterfaces.iControlMode2 = nullptr;
        m_yarpInterfaces.iPositionControl = nullptr;
        m_yarpInterfaces.iPositionDirect = nullptr;
        m_yarpInterfaces.iVelocityControl = nullptr;
        m_yarpInterfaces.iTorqueControl = nullptr;
        m_yarpInterfaces.iPWMControl = nullptr;
        m_yarpInterfaces.iCurrentControl = nullptr;
        m_yarpInterfaces.iEncoders = nullptr;
        m_yarpInterfaces.iControlLimits2 = nullptr;
        m_yarpInterfaces.iPidControl = nullptr;
        //  Close the device (which deletes the interfaces it allocated)
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

    // Explicitly set the velocity representation
    m_kinDynComp->setFrameVelocityRepresentation(iDynTree::MIXED_REPRESENTATION);

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
        Log::getSingleton().errorAppend("\nPossible causes: file not found, or the joint ");
        Log::getSingleton().errorAppend("list contains an entry not present in the urdf model.");
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
T* RobotInterface::getInterfaceFromTemplate(T*& interface)
{
    if (!interface) {
        // Blocks which require the RemoteControlBoardRemapper need to retain / release it
        // in their initialization / terminate phase;
        // assert(m_robotDevice);
        if (!m_robotDevice) {
            Log::getSingleton().error("The RemoteControlBoardRemapper has not been initialized. ");
            Log::getSingleton().errorAppend("You need to retain the CB device in the initialize() method.");
            // Return an empty weak pointer
            return nullptr;
        }

        // Ask the interface from the device
        if (!m_robotDevice->view(interface)) {
            // Return an empty weak_ptr
            return nullptr;
        }
    }
    return interface;
}
