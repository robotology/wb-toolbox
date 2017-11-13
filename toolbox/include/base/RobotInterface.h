#ifndef WBT_ROBOTINTERFACE_H
#define WBT_ROBOTINTERFACE_H

#include "Configuration.h"
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <cassert>
#include <unordered_map>

namespace yarp {
    namespace dev {
        class PolyDriver;
        class IPositionControl;
        class IPositionDirect;
        class IVelocityControl;
        class ITorqueControl;
        class IPWMControl;
        class IControlMode2;
        class ICurrentControl;
        class IEncoders;
        class IControlLimits2;
        class IPidControl;
    }
}

namespace iDynTree {
    class KinDynComputations;
}

namespace wbt {
    class RobotInterface;
    struct YarpDevices;

    typedef int jointIdx_yarp;
    typedef int jointIdx_iDynTree;
    typedef unsigned cb_idx;
    typedef std::unordered_map<jointIdx_iDynTree, std::pair<cb_idx, jointIdx_yarp>> JointsMapIndex;
    typedef std::unordered_map<std::string, std::pair<cb_idx, jointIdx_yarp>> JointsMapString;
}

/**
 * \struct wbt::YarpDevices RobotInterface.h
 *
 * This struct contains shared_ptrs to the devices which are (lazy) asked from the blocks.
 *
 * @note The shared_ptr is owned only by wbt::RobotInterface. All the blocks will receive a
 *       weak_ptr.
 * @remark Right now only asking / setting the whole joint set of the current Configuration is
 *         supported. If in the future the support of operating on a subset will be implemented,
 *         IPositionControl2 and IVelocityControl2 should be implemented. For the time being,
 *         a possible workaround for this situation is creating a new configuration block
 *         containing only the reduced set in a deeper-hierarchy Simulink's subsystem.
 * @see RobotInterface::getDevice
 */
struct wbt::YarpDevices
{
    std::shared_ptr<yarp::dev::IControlMode2> iControlMode2;
    std::shared_ptr<yarp::dev::IPositionControl> iPositionControl;
    std::shared_ptr<yarp::dev::IPositionDirect> iPositionDirect;
    std::shared_ptr<yarp::dev::IVelocityControl> iVelocityControl;
    std::shared_ptr<yarp::dev::ITorqueControl> iTorqueControl;
    std::shared_ptr<yarp::dev::IPWMControl> iPWMControl;
    std::shared_ptr<yarp::dev::ICurrentControl> iCurrentControl;
    std::shared_ptr<yarp::dev::IEncoders> iEncoders;
    std::shared_ptr<yarp::dev::IControlLimits2> iControlLimits2;
    std::shared_ptr<yarp::dev::IPidControl> iPidControl;
};

// TODO o pensare come evitare di avere due conf con es position e torque nei setref con lo stesso robot.
// e' un casino -> aggiungere max un warning o solo documentazione.
// Solo un setReference attivo per configuration.Se ci sono due setReference con lo stesso
// set di giunti attivi contemporaneamente. Oppure due stessi blocchi setReference con lo stesso blocco config.

/**
 * \class wbt::RobotInterface RobotInterface.h
 *
 * This class holds the configuration used by one or more blocks, and all the objects to operate
 * with the specified robot (real or model).
 *
 * @see wbt::Configuration
 * @see wbt::YarpDevices
 * @see iDynTree::KinDynComputations
 */
class wbt::RobotInterface
{
private:
    std::unique_ptr<yarp::dev::PolyDriver> m_robotDevice;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynComp;
    wbt::YarpDevices m_yarpDevices;

    // Maps used to store infos about yarp's and idyntree's internal joint indexing
    std::shared_ptr<JointsMapIndex> m_jointsMapIndex;
    std::shared_ptr<JointsMapString> m_jointsMapString;
    // std::unordered_map<std::pair<cb_idx, yarp_idx>, joint_name> m_yarp2joint;

    // Configuration from Simulink Block's parameters
    const wbt::Configuration m_config;

    // Counters for resource allocation / deallocation
    unsigned m_robotDeviceCounter;

    // INITIALIZATION HELPERS
    // ======================

    /**
     * Initialize the iDynTree::KinDynComputations with the information contained
     * in wbt::Configuration. It finds the urdf file and stores the object to operate on it.
     * If the joint list contained in RobotInterface::m_config is not complete, it loads a
     * reduced model of the robot
     *
     * @return True if success
     */
    bool initializeModel();

    /**
     * Configure a RemoteControlBoardRemapper device in order to allow
     * interfacing the toolbox with the robot (real or in Gazebo).
     *
     * @return True if success
     */
    bool initializeRemoteControlBoardRemapper();

    // OTHER PRIVATE METHODS
    // =====================

    /**
     * Gather a device from the RemoteControlBoardRemapper as a \c weak_ptr
     *
     * @param  device The RemoteControlBoardRemapper device
     * @return        The dynamic(ally)_cast device
     * @tparam T      The type of the retured device
     */
    template <typename T>
    std::weak_ptr<T> getInterfaceFromTemplate(std::shared_ptr<T> device);

    /**
     * Creates the map between joints (specified as either names or idyntree indices) and
     * their YARP representation, which consist in a pair: Control Board index and joint index inside
     * the its Control Board.
     *
     * @see getJointsMapString
     * @see getJointsMapIndex
     *
     * @return True if the map has been created successfully
     */
    bool mapDoFs();
public:

    // CONSTRUCTOR / DESTRUCTOR
    // ========================

    RobotInterface() = delete;
    RobotInterface(const wbt::Configuration& c);
    ~RobotInterface();

    // GET METHODS
    // ===========

    /**
     * Get the current configuration
     *
     * @return A reference of the Configuration object containing the current configuraton
     */
    const wbt::Configuration& getConfiguration() const;

    /**
     * Get the map between model joint namesand the YARP representation (Control Board and
     * joint index)
     *
     * @return The joint map
     */
    const std::shared_ptr<JointsMapString> getJointsMapString();

    /**
     * Get the map between model joint indices and the YARP representation (Control Board and
     * joint index)
     *
     * @return The joint map
     */
    const std::shared_ptr<JointsMapIndex> getJointsMapIndex();

    /**
     * Get the object to operate on the configured model
     *
     * @return A \c shared_ptr to the KinDynComputations object
     */
    const std::shared_ptr<iDynTree::KinDynComputations> getKinDynComputations();

    /**
     * Get a \c weak_ptr to an interface from the RemoteControlBoardRemapper
     *
     * param interface [out] A \c weak_ptr to the interface
     * @return               True if the \c weak_ptr is valid
     * @tparam T             The type of interface
     */
    template <typename T>
    bool getInterface(std::weak_ptr<T>& interface);

    // LAZY EVALUATION
    // ===============

    /**
     * Handles the internal counter for using the RemoteControlBoardRemapper
     *
     * @attention All the blocks which need to use any of the interfaces provided by
     *            wbt::YarpDevices must call this function in their initialize() method.
     * @see releaseRemoteControlBoardRemapper
     *
     * @return True if success
     */
    bool retainRemoteControlBoardRemapper();

    /**
     * Handles the internal counter for using the RemoteControlBoardRemapper. After the call
     * from the last instance which retained the object, the RemoteControlBoardRemapper and all
     * the allocated drivers get destroyed.
     *
     * @note On the contrary of retainRemoteControlBoardRemapper, this method is already
     *       called in wbt::~WBBlock
     * @see retainRemoteControlBoardRemapper
     *
     * @return True if success
     */
    bool releaseRemoteControlBoardRemapper();
};

#endif /* end of include guard: WBT_ROBOTINTERFACE_H */
