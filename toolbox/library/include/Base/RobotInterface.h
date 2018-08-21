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
        class IControlMode;
        class ICurrentControl;
        class IEncoders;
        class IMotorEncoders;
        class IControlLimits;
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
     * @brief Get the object to operate on the configured model
     *
     * @return A `shared_ptr` to the KinDynComputations object.
     */
    const std::shared_ptr<iDynTree::KinDynComputations> getKinDynComputations();

    /**
     * @brief Get a Yarp interface
     *
     * The interface is lazy-evaluated. The handling of the memory is not responsibility of the
     * caller. It is handled internally.
     *
     * @param[out] interface The object that will contain the pointer to the interface.
     * @return True for success, false otherwise.
     */
    template <typename T>
    bool getInterface(T*& interface);
};

// Specialize the getInterface template
namespace wbt {
    template <>
    bool RobotInterface::getInterface(yarp::dev::IControlMode*& interface);
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
    bool RobotInterface::getInterface(yarp::dev::IMotorEncoders*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::IControlLimits*& interface);
    template <>
    bool RobotInterface::getInterface(yarp::dev::IPidControl*& interface);
} // namespace wbt

#endif // WBT_ROBOTINTERFACE_H
