/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_ROBOTINTERFACE_H
#define WBT_ROBOTINTERFACE_H

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

namespace yarp {
    namespace dev {
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
    class Configuration;

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
 * @brief Class for handling model and robot resources
 *
 * This class is a wrapper of yarp::dev::RemoteControlBoardRemapper and
 * iDynTree::KinDynComputations. By filling information in a wbt::Configuration object, it provides
 * a simple initialization and combined usage of these two resources.
 *
 * @see wbt::Configuration
 */
class wbt::RobotInterface
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    // CONSTRUCTOR / DESTRUCTOR
    // ========================

    RobotInterface() = delete;
    RobotInterface(const wbt::Configuration& config);
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
