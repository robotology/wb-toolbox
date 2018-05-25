/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "Configuration.h"

#include <algorithm>

using namespace wbt;

class Configuration::impl
{
public:
    std::string confKey;
    std::string robotName;
    std::string urdfFile;
    std::string localName;
    std::vector<std::string> controlledJoints;
    std::vector<std::string> controlBoardsNames;
    std::array<double, 3> gravityVector;
    size_t dofs = 0;

    impl* clone() { return new impl(*this); }
};

Configuration::Configuration(const std::string& confKey)
    : pImpl{new impl()}
{
    pImpl->confKey = confKey;
}

Configuration::Configuration(const wbt::Configuration& other)
    : pImpl{other.pImpl->clone()}
{}

// Defining the destructor as default here in the cpp avoids the usage
// of a custom pimpl deleter
Configuration::~Configuration() = default;

// SET METHODS
// ===========

void Configuration::setParameters(const std::string& robotName,
                                  const std::string& urdfFile,
                                  const std::vector<std::string>& controlledJoints,
                                  const std::vector<std::string>& controlBoardsNames,
                                  const std::string& localName,
                                  const std::array<double, 3>& gravityVector)
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
    pImpl->robotName = robotName;
}

void Configuration::setUrdfFile(const std::string& urdfFile)
{
    pImpl->urdfFile = urdfFile;
}

void Configuration::setControlledJoints(const std::vector<std::string>& controlledJoints)
{
    pImpl->controlledJoints = controlledJoints;
    pImpl->dofs = controlledJoints.size();
}

void Configuration::setControlBoardsNames(const std::vector<std::string>& controlBoardsNames)
{
    pImpl->controlBoardsNames = controlBoardsNames;
}

void Configuration::setLocalName(const std::string& localName)
{
    pImpl->localName = localName;

    // Add the leading "/" if missing
    if (pImpl->localName.compare(0, 1, "/")) {
        pImpl->localName = "/" + pImpl->localName;
    }
}

const std::string Configuration::getUniqueId() const
{
    std::string uniqueId(pImpl->confKey);

    // Remove spaces
    auto it = std::remove(uniqueId.begin(), uniqueId.end(), ' ');
    uniqueId.erase(it, uniqueId.end());

    // Remove '/'
    it = std::remove(uniqueId.begin(), uniqueId.end(), '/');
    uniqueId.erase(it, uniqueId.end());

    return uniqueId;
}

void Configuration::setGravityVector(const std::array<double, 3>& gravityVector)
{
    pImpl->gravityVector = gravityVector;
}

// GET METHODS
// ===========

const std::string& Configuration::getRobotName() const
{
    return pImpl->robotName;
}

const std::string& Configuration::getUrdfFile() const
{
    return pImpl->urdfFile;
}

const std::vector<std::string>& Configuration::getControlledJoints() const
{
    return pImpl->controlledJoints;
}

const std::vector<std::string>& Configuration::getControlBoardsNames() const
{
    return pImpl->controlBoardsNames;
}

const std::string& Configuration::getLocalName() const
{
    return pImpl->localName;
}

const std::array<double, 3>& Configuration::getGravityVector() const
{
    return pImpl->gravityVector;
}

const size_t& Configuration::getNumberOfDoFs() const
{
    return pImpl->dofs;
}

const std::string Configuration::getConfKey() const
{
    return pImpl->confKey;
}

// OTHER METHODS
// =============

bool Configuration::isValid() const
{
    bool status = !pImpl->robotName.empty() && !pImpl->urdfFile.empty() && !pImpl->localName.empty()
                  && !pImpl->controlledJoints.empty() && !pImpl->controlBoardsNames.empty()
                  && !pImpl->gravityVector.empty() && pImpl->dofs > 0;
    return status;
}

// OPERATORS OVERLOADING
// =====================

bool Configuration::operator==(const Configuration& config) const
{
    return this->pImpl->robotName == config.pImpl->robotName
           && this->pImpl->urdfFile == config.pImpl->urdfFile
           && this->pImpl->localName == config.pImpl->localName
           && this->pImpl->controlledJoints == config.pImpl->controlledJoints
           && this->pImpl->controlBoardsNames == config.pImpl->controlBoardsNames
           && this->pImpl->gravityVector == config.pImpl->gravityVector
           && this->pImpl->dofs == config.pImpl->dofs;
}
