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

    using JointIndex_Yarp = int;
    using JointIndex_iDynTree = int;
    using JointName = std::string;
    using ControlBoardIndex = unsigned;
    using JointIndexInControlBoard = unsigned;

    using JointIndexToYarpMap =
        std::unordered_map<JointIndex_iDynTree, std::pair<ControlBoardIndex, JointIndex_Yarp>>;
    using JointNameToYarpMap =
        std::unordered_map<JointName, std::pair<ControlBoardIndex, JointIndex_Yarp>>;
    using JointNameToIndexInControlBoardMap =
        std::unordered_map<JointName, JointIndexInControlBoard>;

    using ControlBoardMaxIndex = unsigned;
    using ControlBoardIndexLimit = std::unordered_map<ControlBoardIndex, ControlBoardMaxIndex>;
} // namespace wbt

/**
 * @brief A container for yarp interfaces pointers
 *
 * The pointers are lazy-asked by wbt::RobotInterface and stored in this struct.
 *
 * @remark Right now only operating on the whole joint set of the current Configuration is
 *         supported. If in the future the support of operating on a subset will be implemented,
 *         `IPositionControl2` and `IVelocityControl2` should be implemented. For the time being,
 *         a possible workaround for this situation is creating a new configuration block
 *         containing only the reduced set in a deeper-hierarchy Simulink's subsystem.
 * @see RobotInterface::getInterface
 */
struct wbt::YarpInterfaces
{
    yarp::dev::IControlMode2* iControlMode2 = nullptr;
    yarp::dev::IPositionControl* iPositionControl = nullptr;
    yarp::dev::IPositionDirect* iPositionDirect = nullptr;
    yarp::dev::IVelocityControl* iVelocityControl = nullptr;
    yarp::dev::ITorqueControl* iTorqueControl = nullptr;
    yarp::dev::IPWMControl* iPWMControl = nullptr;
    yarp::dev::ICurrentControl* iCurrentControl = nullptr;
    yarp::dev::IEncoders* iEncoders = nullptr;
    yarp::dev::IControlLimits2* iControlLimits2 = nullptr;
    yarp::dev::IPidControl* iPidControl = nullptr;
};

/**
 * @brief Class for handling model and robot resources
 *
 * This class is a wrapper of yarp::dev::RemoteControlBoardRemapper and
 * iDynTree::KinDynComputations. By filling information in a wbt::Configuration object, it provides
 * a simple initialization and combined usage of these two resources.
 *
 * @see wbt::Configuration, wbt::YarpInterfaces
 */
class wbt::RobotInterface
{
private:
    std::unique_ptr<yarp::dev::PolyDriver> m_robotDevice;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDynComp;
    wbt::YarpInterfaces m_yarpInterfaces;

    // Maps used to store infos about yarp's and idyntree's internal joint indexing
    std::shared_ptr<JointIndexToYarpMap> m_jointIndexToYarpMap;
    std::shared_ptr<JointNameToYarpMap> m_jointNameToYarpMap;
    std::shared_ptr<JointNameToIndexInControlBoardMap> m_jointNameToIndexInControlBoardMap;
    std::shared_ptr<ControlBoardIndexLimit> m_controlBoardIndexLimit;

    // Configuration from Simulink Block's parameters
    const wbt::Configuration m_config;

    // Counters for resource allocation / deallocation
    unsigned m_robotDeviceCounter;

    // ======================
    // INITIALIZATION HELPERS
    // ======================

    /**
     *
     * @brief Initialize the model
     *
     * Initialize the iDynTree::KinDynComputations with the information contained
     * in wbt::Configuration. It finds from the file system the urdf file and stores the object to
     * operate on it. If the joint list contained in RobotInterface::m_config is not complete, it
     * loads a reduced model of the robot.
     *
     * @return True if success, false otherwise.
     */
    bool initializeModel();

    /**
     * @brief Initialize the remote controlboard remapper
     *
     * Configure a yarp::dev::RemoteControlBoardRemapper device in order to allow
     * interfacing the toolbox with the robot (real or in Gazebo).
     *
     * @return True if success, false otherwise.
     */
    bool initializeRemoteControlBoardRemapper();

    // =====================
    // OTHER PRIVATE METHODS
    // =====================

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
    bool mapDoFs();

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
     * @brief Get the stored configuration
     *
     * @return A reference of the configuration this object refers to.
     */
    const wbt::Configuration& getConfiguration() const;

    /**
     * @brief Get the map between model joint names and the YARP representation (Control Board and
     * joint index)
     *
     * @return The joint map.
     */
    const std::shared_ptr<JointNameToYarpMap> getJointsMapString();

    /**
     * @brief Get the map between model joint indices and the YARP representation (Control Board and
     * joint index)
     *
     * @return The joint map.
     */
    const std::shared_ptr<JointIndexToYarpMap> getJointsMapIndex();

    /**
     * @brief Get the map between model joint names and the index representing their relative
     * ordering inside the controlledJoints vector relative to their ControlBoard
     *
     * @remark For example, if the joints are `j1_cb1 j2_cb1 j3_cb1 j1_cb2 j2_cb2 j1_cb3`, the map
     * links them to `0 1 2 0 1 0`. Note that joints are 0-indexed.
     *
     * @return The joint map.
     */
    const std::shared_ptr<JointNameToIndexInControlBoardMap> getControlledJointsMapCB();

    /**
     * @brief Get the map between the ControlBoard index inside the RemoteControlBoardRemapper
     * and the number of the controlled joints belonging to it
     *
     * @remark For example, if the joints are
     * ```
     * j1_cb1 j2_cb1 j3_cb1 j1_cb2 j2_cb2 j1_cb3
     * ```
     * the generated map is
     * ```
     * {{0,3}{1,2}{2,1}}
     * ```
     * Note that the map key is 0-indexed.
     *
     * @return The control board limit map.
     */
    const std::shared_ptr<ControlBoardIndexLimit> getControlBoardIdxLimit();

    /**
     * @brief Get the object to operate on the configured model
     *
     * @return A `shared_ptr` to the KinDynComputations object.
     */
    const std::shared_ptr<iDynTree::KinDynComputations> getKinDynComputations();

    /**
     * @brief Get a Yarp interface
     *
     * The interface is lazy-evaluated. The handling of the memory is not responbility of the
     * caller. It is handled internally.
     *
     * @param[out] interface The object that will contain the pointer to the interface.
     * @return True for success, false otherwise.
     */
    template <typename T>
    bool getInterface(T*& interface);

    // LAZY EVALUATION
    // ===============

    /**
     * @brief Handle the internal counter for using the `RemoteControlBoardRemapper`.
     *
     * @attention All the blocks which need to use any of the interfaces provided by
     *            wbt::YarpInterfaces must call this function in their Block::initialize method.
     *
     * @return True if success, false otherwise.
     * @see RobotInterface::releaseRemoteControlBoardRemapper
     */
    bool retainRemoteControlBoardRemapper();

    /**
     * @brief Handle the internal counter for using the `RemoteControlBoardRemapper`.
     *
     * After the call from the last instance which retained the object, the
     * `RemoteControlBoardRemapper` and all the allocated drivers get destroyed.
     *
     * @attention All the blocks which need to use any of the interfaces provided by
     *            wbt::YarpInterfaces must call this function in their Block::terminate method.
     *
     * @return True if success, false otherwise.
     * @see RobotInterface::retainRemoteControlBoardRemapper
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
