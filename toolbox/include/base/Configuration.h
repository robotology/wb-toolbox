/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_TOOLBOXCONFIG_H
#define WBT_TOOLBOXCONFIG_H

#include <array>
#include <string>
#include <vector>

namespace wbt {
    class Configuration;
}

/**
 * @brief Store the configuration for whole-body blocks
 *
 * This class stores the content of the `WBToolbox.Configuration` Matlab class and its related
 * _configuration_ block. This in principle is Simulink specific, but nothing prevents using it from
 * C++ for instantiating a wbt::RobotInterface object.
 *
 * Its usage is specific for using yarp::dev::RemoteControlBoardRemapper and
 * iDynTree::KinDynComputations objects contained in the wbt::RobotInterface class.
 *
 * @see wbt::RobotInterface, wbt::WBBlock, wbt::ToolboxSingleton, WBToolbox.Configuration Matlab
 * Class
 */
class wbt::Configuration
{
private:
    std::string m_confKey; ///< Name of the block which this object refers to (unique identifier)
    std::string m_robotName; ///< Name of the robot
    std::string m_urdfFile; ///< Name of the file containing the urdf model
    std::string m_localName; ///< Prefix appended to the opened ports
    std::vector<std::string> m_controlledJoints; ///< Subset of controlled joints
    std::vector<std::string> m_controlBoardsNames; ///< Names of the used ControlBoard names
    std::array<double, 3> m_gravityVector; ///< The gravity vector
    size_t m_dofs; ///< DoFs extracted from Configuration::m_controlBoardsNames vector

public:
    Configuration() = default;
    Configuration(const std::string& confKey);
    ~Configuration() = default;

    // ===========
    // SET METHODS
    // ===========

    /**
     * @brief Initialize the object
     *
     * @param robotName  Name of the robot.
     * @param urdfFile Name of the file containing the urdf model.
     * @param controlledJoints Subset of controlled joints.
     * @param controlBoardsNames Names of the used ControlBoard names.
     * @param localName Prefix appended to the opened ports.
     * @param gravityVector The gravity vector.
     */
    void setParameters(const std::string& robotName,
                       const std::string& urdfFile,
                       const std::vector<std::string>& controlledJoints,
                       const std::vector<std::string>& controlBoardsNames,
                       const std::string& localName,
                       const std::array<double, 3>& gravityVector);

    /**
     * @brief Set the name of the robot
     *
     * @param robotName Name of the robot.
     */
    void setRobotName(const std::string& robotName);

    /**
     * @brief Set the name of the file containing the urdf model
     *
     * @param urdfFile Name of the file containing the urdf model.
     */
    void setUrdfFile(const std::string& urdfFile);

    /**
     * @brief Set the subset of controlled joints
     *
     * @param controlledJoints Subset of controlled joints.
     */
    void setControlledJoints(const std::vector<std::string>& controlledJoints);

    /**
     * @brief Set the names of the used ControlBoard names
     *
     * @param controlBoardsNames Names of the used ControlBoard names.
     */
    void setControlBoardsNames(const std::vector<std::string>& controlBoardsNames);

    /**
     * @brief Set the prefix appended to the opened ports
     *
     * @param localName Prefix appended to the opened ports.
     */
    void setLocalName(const std::string& localName);

    /**
     * @brief Set the gravity vector
     *
     * @param gravityVector The gravity vector.
     */
    void setGravityVector(const std::array<double, 3>& gravityVector);

    // ===========
    // GET METHODS
    // ===========

    /**
     * @brief Get the name of the robot
     *
     * @return Name of the robot.
     */
    const std::string& getRobotName() const;

    /**
     * @brief Get the name of the file containing the urdf model
     *
     * @return Name of the file containing the urdf model.
     */
    const std::string& getUrdfFile() const;

    /**
     * @brief Get the subset of controlled joints
     *
     * @return Subset of controlled joints.
     */
    const std::vector<std::string>& getControlledJoints() const;

    /**
     * @brief Get the names of the used ControlBoard names
     *
     * @return Names of the used ControlBoard names.
     */
    const std::vector<std::string>& getControlBoardsNames() const;

    /**
     * @brief Get the prefix appended to the opened ports. A leading "/" is always present.
     *
     * @return Prefix appended to the opened ports.
     */
    const std::string& getLocalName() const;

    /**
     * @brief Generate a unique identifier
     *
     * Get a string with a unique identifier generated from the name of the config block from
     * Simulink. It might be useful when yarp ports must have a unique prefix (e.g. two
     * RemoteControlBoardRemappers which share a ControlBoard)
     *
     * @return The unique identifier.
     * @see setLocalName
     */
    const std::string getUniqueId() const;

    /**
     * @brief Get the gravity vector
     *
     * @return The gravity vector.
     */
    const std::array<double, 3>& getGravityVector() const;

    /**
     * Get the configured number of DoFs
     *
     * @return The configured number of DoFs.
     */
    const size_t& getNumberOfDoFs() const;

    /**
     * @brief Get the unique identifier of this configuration object
     *
     * @return The unique identifier.
     */
    const std::string getConfKey() const { return m_confKey; }

    // =============
    // OTHER METHODS
    // =============

    /**
     * @brief Check if the congiguration is valid
     *
     * I.e. all the required fields have been stored successfully.
     *
     * @return True if the configuration is valid.
     */
    bool isValid() const;

    // =====================
    // OPERATORS OVERLOADING
    // =====================

    bool operator==(const Configuration& config) const;
};

#endif // WBT_TOOLBOXCONFIG_H
