/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_ROBOTINTERFACE_H
#define WBT_ROBOTINTERFACE_H

#include "Configuration.h"

#include <cassert>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

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
    } // namespace dev
} // namespace yarp

namespace iDynTree {
    class KinDynComputations;
}

namespace wbt {
    class RobotInterface;
    struct YarpInterfaces;

    typedef int jointIdx_yarp;
    typedef int jointIdx_iDynTree;
    typedef unsigned cb_idx;
    typedef unsigned max_cb_idx;
    typedef unsigned controlledJointIdxCB;
    typedef std::unordered_map<jointIdx_iDynTree, std::pair<cb_idx, jointIdx_yarp>> JointsMapIndex;
    typedef std::unordered_map<std::string, std::pair<cb_idx, jointIdx_yarp>> JointsMapString;
    typedef std::unordered_map<std::string, controlledJointIdxCB> ControlledJointsMapCB;
    typedef std::unordered_map<cb_idx, max_cb_idx> ControlBoardIdxLimit;
} // namespace wbt

/**
 * \struct wbt::YarpInterfaces RobotInterface.h
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
struct wbt::YarpInterfaces
{
    yarp::dev::IControlMode2* iControlMode2;
    yarp::dev::IPositionControl* iPositionControl;
    yarp::dev::IPositionDirect* iPositionDirect;
    yarp::dev::IVelocityControl* iVelocityControl;
    yarp::dev::ITorqueControl* iTorqueControl;
    yarp::dev::IPWMControl* iPWMControl;
    yarp::dev::ICurrentControl* iCurrentControl;
    yarp::dev::IEncoders* iEncoders;
    yarp::dev::IControlLimits2* iControlLimits2;
    yarp::dev::IPidControl* iPidControl;

    YarpInterfaces()
        : iControlMode2(nullptr)
        , iPositionControl(nullptr)
        , iPositionDirect(nullptr)
        , iVelocityControl(nullptr)
        , iTorqueControl(nullptr)
        , iPWMControl(nullptr)
        , iCurrentControl(nullptr)
        , iEncoders(nullptr)
        , iControlLimits2(nullptr)
        , iPidControl(nullptr)
    {}
};

/**
 * \class wbt::RobotInterface RobotInterface.h
 *
 * This class holds the configuration used by one or more blocks, and all the objects to operate
 * with the specified robot (real or model).
 *
 * @see wbt::Configuration
 * @see wbt::YarpInterfaces
 * @see iDynTree::KinDynComputations
 */
class wbt::RobotInterface
{
private:
    std::unique_ptr<yarp::dev::PolyDriver> m_robotDevice;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynComp;
    wbt::YarpInterfaces m_yarpInterfaces;

    // Maps used to store infos about yarp's and idyntree's internal joint indexing
    std::shared_ptr<JointsMapIndex> m_jointsMapIndex;
    std::shared_ptr<JointsMapString> m_jointsMapString;
    std::shared_ptr<ControlledJointsMapCB> m_controlledJointsMapCB;
    std::shared_ptr<ControlBoardIdxLimit> m_controlBoardIdxLimit;

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
     * Creates the map between joints (specified as either names or idyntree indices) and
     * their YARP representation, which consist in a pair: Control Board index and joint index
     * inside the its Control Board.
     *
     * @see getJointsMapString
     * @see getJointsMapIndex
     *
     * @return True if the map has been created successfully
     */
    bool mapDoFs();

    /**
     * Create a RemoteControlBoard object for a given remoteName
     *
     * @see mapDoFs
     *
     * @param  remoteName   [in]  Name of the remote from which the remote control board is be
     * initialized
     * @param  controlBoard [out] Smart pointer to the allocated remote control board
     * @return                    True if success
     */
    bool getSingleControlBoard(const std::string& remoteName,
                               std::unique_ptr<yarp::dev::PolyDriver>& controlBoard);

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
     * Get the map between model joint names and the YARP representation (Control Board and
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
     * Get the map between model joint names and the index representing their relative ordering
     * inside the controlledJoints vector relative to their ControlBoard.
     *
     * @note For example, if the joints are
     * \verbatim j1_cb1 j2_cb1 j3_cb1 j1_cb2 j2_cb2 j1_cb3 \endverbatim, the map links them to
     * \verbatim 0 1 2 0 1 0 \end
     *
     * @return The joint map
     */
    const std::shared_ptr<ControlledJointsMapCB> getControlledJointsMapCB();

    /**
     * Get the map between the ControlBoard index inside the RemoteControlBoardRemapper
     * and the number of the controlled joints which belongs to it.
     *
     * @note For example, if the joints are
     * \verbatim j1_cb1 j2_cb1 j3_cb1 j1_cb2 j2_cb2 j1_cb3 \endverbatim, the generated map is
     * \verbatim  {{0,3}{1,2}{2,1}} \endverbatim
     * Note that the map key is 0-indexed.
     *
     * @return The control board limit map
     */
    const std::shared_ptr<ControlBoardIdxLimit> getControlBoardIdxLimit();

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
    bool getInterface(T*& interface);

    // LAZY EVALUATION
    // ===============

    /**
     * Handles the internal counter for using the RemoteControlBoardRemapper
     *
     * @attention All the blocks which need to use any of the interfaces provided by
     *            wbt::YarpInterfaces must call this function in their initialize() method.
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

// Specialize the getInterface template
namespace wbt {
    template <>
    bool RobotInterface::getInterface(yarp::dev::IControlMode2*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::IPositionControl*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::IPositionDirect*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::IVelocityControl*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::ITorqueControl*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::IPWMControl*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::ICurrentControl*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::IEncoders*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::IControlLimits2*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::IPidControl*& interface);
} // namespace wbt

#endif // WBT_ROBOTINTERFACE_H
