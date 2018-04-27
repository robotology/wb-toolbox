/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "ToolboxSingleton.h"
#include "Configuration.h"
#include "Log.h"
#include "Parameters.h"
#include "RobotInterface.h"

#include <array>
#include <cassert>
#include <ostream>
#include <stddef.h>
#include <string>
#include <vector>

using namespace wbt;

bool fillConfiguration(std::shared_ptr<wbt::Configuration>& configurationPtr,
                       const wbt::Parameters& parameters);

// CONSTRUCTOR / DESTRUCTOR
// ========================

ToolboxSingleton::ToolboxSingleton() {}
ToolboxSingleton::~ToolboxSingleton() {}

// UTILITIES
// =========

int ToolboxSingleton::numberOfDoFs(const std::string& confKey)
{
    if (!isKeyValid(confKey))
        return -1;
    else
        return m_interfaces[confKey]->getConfiguration().getNumberOfDoFs();
}

bool ToolboxSingleton::isKeyValid(const std::string& confKey) const
{
    if (m_interfaces.find(confKey) != m_interfaces.end()) {
        if (m_interfaces.at(confKey))
            return true;
        else
            return false;
    }
    else {
        return false;
    }
}

// GET METHODS
// ===========

ToolboxSingleton& ToolboxSingleton::sharedInstance()
{
    static ToolboxSingleton instance;
    return instance;
}

const Configuration& ToolboxSingleton::getConfiguration(const std::string& confKey) const
{
    return getRobotInterface(confKey)->getConfiguration();
}

const std::shared_ptr<RobotInterface>
ToolboxSingleton::getRobotInterface(const std::string& confKey) const
{
    if (!isKeyValid(confKey)) {
        return nullptr;
    }

    return m_interfaces.at(confKey);
}

const std::shared_ptr<iDynTree::KinDynComputations>
ToolboxSingleton::getKinDynComputations(const std::string& confKey) const
{
    if (!isKeyValid(confKey)) {
        return nullptr;
    }

    return m_interfaces.at(confKey)->getKinDynComputations();
}

// TOOLBOXSINGLETON CONFIGURATION
// ==============================

bool ToolboxSingleton::storeConfiguration(const Configuration& config)
{
    if (!config.isValid()) {
        return false;
    }

    const std::string confKey = config.getConfKey();

    // Add the new Configuration object and override an existing key if it already exist.
    // Note: Simulink doesn't flush memory unless Matlab is closed, and static objects stay in
    // memory.
    //       This may cause problems if the config block's mask is changed after the first
    //       compilation.
    if (m_interfaces.find(confKey) == m_interfaces.end()) {
        m_interfaces[confKey] = std::make_shared<RobotInterface>(config);
        return static_cast<bool>(m_interfaces[confKey]);
    }

    if (!(m_interfaces[confKey]->getConfiguration() == config)) {
        assert(m_interfaces[confKey]);

        // Delete the old configuration (calling the destructor for cleaning garbage)
        m_interfaces[confKey].reset();
        m_interfaces.erase(confKey);

        // Allocate a new configuration
        m_interfaces[confKey] = std::make_shared<RobotInterface>(config);
        return static_cast<bool>(m_interfaces[confKey]);
    }

    return true;
}

bool fillConfiguration(std::shared_ptr<wbt::Configuration>& configurationPtr,
                       const wbt::Parameters& parameters)
{
    bool ok = true;

    std::string robotName;
    std::string urdfFile;
    std::string localName;
    std::string confBlockName;
    std::vector<double> gravityVector;
    std::vector<std::string> controlledJoints;
    std::vector<std::string> controlBoardsNames;

    ok = ok && parameters.getParameter("RobotName", robotName);
    ok = ok && parameters.getParameter("UrdfFile", urdfFile);
    ok = ok && parameters.getParameter("LocalName", localName);
    ok = ok && parameters.getParameter("ControlledJoints", controlledJoints);
    ok = ok && parameters.getParameter("ControlBoardsNames", controlBoardsNames);
    ok = ok && parameters.getParameter("GravityVector", gravityVector);
    ok = ok && parameters.getParameter("ConfBlockName", confBlockName);

    // Populate the Configuration object
    // =================================

    configurationPtr.reset(new wbt::Configuration(confBlockName));
    configurationPtr->setRobotName(robotName);
    configurationPtr->setUrdfFile(urdfFile);
    configurationPtr->setControlledJoints(controlledJoints);
    configurationPtr->setControlBoardsNames(controlBoardsNames);
    configurationPtr->setLocalName(localName);

    std::array<double, 3> gravityArray;
    for (size_t i = 0; i < 3; ++i) {
        gravityArray[i] = gravityVector[i];
    }
    configurationPtr->setGravityVector(gravityArray);

    return ok;
}

bool ToolboxSingleton::storeConfiguration(const wbt::Parameters& parameters)
{
    if (!Parameters::containConfigurationData(parameters)) {
        wbtError << "Passed Parameters object does not contain the right data for initializing a "
                 << "Configuration object.";
        return false;
    }

    std::shared_ptr<Configuration> configurationPtr = nullptr;
    if (!fillConfiguration(configurationPtr, parameters)) {
        wbtError << "Failed to fill the configuration with input parameters.";
        return false;
    }

    if (!configurationPtr || !configurationPtr->isValid()) {
        wbtError << "Parsed Configuration object is not valid.";
        return false;
    }

    // Insert the configuration into the Toolbox Singleton
    if (!storeConfiguration(*configurationPtr)) {
        wbtError << "Failed to store the given configuration in the ToolboxSingleton.";
        return false;
    }

    return true;
}

void ToolboxSingleton::eraseConfiguration(const std::string& confKey)
{
    m_interfaces.erase(confKey);
}
