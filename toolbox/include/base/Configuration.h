#ifndef WBT_TOOLBOXCONFIG_H
#define WBT_TOOLBOXCONFIG_H

#include <vector>
#include <string>

namespace wbt {
    class Configuration;
}

/**
 * \class Configuration Configuration.h
 *
 * This class stores in a C++ object the content of the WBToolboxConfig Matlab class.
 *
 * @see WBToolboxConfig Matlab Class
 * @see RobotInterface
 */
class wbt::Configuration
{
// TODO: check how localName is used
private:
    std::string m_robotName; ///< Name of the robot
    std::string m_urdfFile;  ///< Name of the file containing the urdf model
    std::string m_localName; ///< Prefix appended to the opened ports
    std::vector<std::string> m_controlledJoints;   ///< Subset of controlled joints
    std::vector<std::string> m_controlBoardsNames; ///< Names of the used ControlBoard names
    std::vector<double> m_gravityVector; ///< The gravity vector
    size_t m_dofs; //< DoFs extracted my m_controlBoardsNames vector

public:
    Configuration();
    ~Configuration() = default;

    // SET METHODS
    // ===========

    /**
     * Initialize the Configuration object
     *
     * @param robotName          Name of the robot
     * @param urdfFile           Name of the file containing the urdf model
     * @param controlledJoints   Subset of controlled joints
     * @param controlBoardsNames Names of the used ControlBoard names
     * @param localName          Prefix appended to the opened ports
     * @param gravityVector      The gravity vector
     */
    void setParameters(std::string robotName,
                       std::string urdfFile,
                       std::vector<std::string> controlledJoints,
                       std::vector<std::string> controlBoardsNames,
                       std::string localName,
                       std::vector<double> gravityVector);

    /**
     * Set the name of the robot
     *
     * @param robotName Name of the robot
     */
    void setRobotName(const std::string& robotName);

    /**
     * Set the name of the file containing the urdf model
     *
     * @param urdfFile Name of the file containing the urdf model
     */
    void setUrdfFile(const std::string& urdfFile);

    /**
     * Set the subset of controlled joints
     * @param controlledJoints Subset of controlled joints
     */
    void setControlledJoints(const std::vector<std::string>& controlledJoints);

    /**
     * Set the names of the used ControlBoard names
     * @param controlBoardsNames Names of the used ControlBoard names
     */
    void setControlBoardsNames(const std::vector<std::string>& controlBoardsNames);

    /**
     * Set the prefix appended to the opened ports
     * @param localName Prefix appended to the opened ports
     */
    void setLocalName(const std::string& localName);

    /**
     * Set the gravity vector
     * @param gravityVector The gravity vector
     */
    void setGravityVector(const std::vector<double>& gravityVector);

    // GET METHODS
    // ===========

    /**
     * Set the name of the robot
     *
     * @return Name of the robot
     */
    const std::string& getRobotName() const;

    /**
     * Set the name of the file containing the urdf model
     *
     * @return Name of the file containing the urdf model
     */
    const std::string& getUrdfFile() const;

    /**
     * Set the subset of controlled joints
     *
     * @return Subset of controlled joints
     */
    const std::vector<std::string>& getControlledJoints() const;

    /**
     * Set the names of the used ControlBoard names
     *
     * @return Names of the used ControlBoard names
     */
    const std::vector<std::string>& getControlBoardsNames() const;

    /**
     * Set the prefix appended to the opened ports
     *
     * @return Prefix appended to the opened ports
     */
    const std::string& getLocalName() const;

    /**
     * Set the gravity vector
     *
     * @return The gravity vector
     */
    const std::vector<double>& getGravityVector() const;

    /**
     * Get the configured number of DoFs
     *
     * @return The configured number of DoFs
     */
    const size_t& getNumberOfDoFs() const;

    // OTHER METHODS
    // =============

    /**
     * Checks if the stored configuration is valid, i.e. all the required fields have
     * been saved successfully
     *
     * @return True if the configuration is valid
     */
    bool isValid() const;

    // OPERATORS OVERLOADING
    // =====================

    bool operator==(const Configuration &config) const;
};

#endif // WBT_TOOLBOXCONFIG_H
