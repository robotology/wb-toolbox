#include "Configuration.h"

using namespace wbt;

Configuration::Configuration()
: m_dofs(0)
{}

// SET METHODS
// ===========

void Configuration::setParameters(std::string robotName,
                                  std::string urdfFile,
                                  std::vector<std::string> controlledJoints,
                                  std::vector<std::string> controlBoardsNames,
                                  std::string localName,
                                  std::vector<double> gravityVector)
{
    setRobotName(robotName);
    setUrdfFile(urdfFile);
    setControlledJoints(controlledJoints);
    setControlBoardsNames(controlBoardsNames);
    setLocalName(localName);
    setGravityVector(gravityVector);
}

void Configuration::setRobotName(const std::string& robotName)
{
    m_robotName = robotName;
}

void Configuration::setUrdfFile(const std::string& urdfFile)
{
    m_urdfFile = urdfFile;
}

void Configuration::setControlledJoints(const std::vector<std::string>& controlledJoints)
{
    m_controlledJoints = controlledJoints;
    m_dofs = controlledJoints.size();
}

void Configuration::setControlBoardsNames(const std::vector<std::string>& controlBoardsNames)
{
    m_controlBoardsNames = controlBoardsNames;
}

void Configuration::setLocalName(const std::string& localName)
{
    m_localName = localName;
}

void Configuration::setGravityVector(const std::vector<double>& gravityVector)
{
    m_gravityVector = gravityVector;
}

// GET METHODS
// ===========

const std::string& Configuration::getRobotName() const
{
    return m_robotName;
}

const std::string& Configuration::getUrdfFile() const
{
    return m_urdfFile;
}

const std::vector<std::string>& Configuration::getControlledJoints() const
{
    return m_controlledJoints;
}

const std::vector<std::string>& Configuration::getControlBoardsNames() const
{
    return m_controlBoardsNames;
}

const std::string& Configuration::getLocalName() const
{
    return m_localName;
}

const std::vector<double>& Configuration::getGravityVector() const
{
    return m_gravityVector;
}

const size_t& Configuration::getNumberOfDoFs() const
{
    return m_dofs;
}

// OTHER METHODS
// =============

bool Configuration::isValid() const
{
    bool status =  !m_robotName.empty() &&
                   !m_urdfFile.empty() &&
                   !m_localName.empty() &&
                   !m_controlledJoints.empty() &&
                   !m_controlBoardsNames.empty() &&
                   !m_gravityVector.empty() &&
                   m_dofs > 0;
    return status;
}

// OPERATORS OVERLOADING
// =====================

bool Configuration::operator==(const Configuration &config) const
{
    return this->m_robotName == config.m_robotName &&
           this->m_urdfFile  == config.m_urdfFile &&
           this->m_localName == config.m_localName &&
           this->m_controlledJoints   == config.m_controlledJoints &&
           this->m_controlBoardsNames == config.m_controlBoardsNames &&
           this->m_gravityVector == config.m_gravityVector &&
           this->m_dofs == config.m_dofs;
}
