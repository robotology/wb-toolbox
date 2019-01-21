/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_WHOLEBODYSINGLETON_H
#define WBT_WHOLEBODYSINGLETON_H

#include <memory>
#include <string>
#include <unordered_map>

namespace wbt {
    namespace base {
        class WholeBodySingleton;
        class RobotInterface;
        class Configuration;
    } // namespace base
} // namespace wbt

namespace blockfactory {
    namespace core {
        class Parameters;
    } // namespace core
} // namespace blockfactory

namespace iDynTree {
    class KinDynComputations;
}

/**
 * This class handles and contains configuration and devices used by all the blocks
 * allocated by a running model. It is possible obtaining a singleton of this class
 * by calling WholeBodySingleton::sharedInstance.
 *
 * @see wbt::RobotInterface
 * @see wbt::Configuration
 * @see wbt::yarpDevices
 *
 */
class wbt::base::WholeBodySingleton
{
private:
    /// Object that stores all the configurations labelled by the name of the Simulink Block's
    /// name.
    /// @see wbt::RobotInterface
    std::unordered_map<std::string, std::weak_ptr<wbt::base::RobotInterface>> m_interfaces;

public:
    // CONSTRUCTOR / DESTRUCTOR
    // ========================

    WholeBodySingleton();
    ~WholeBodySingleton();

    WholeBodySingleton(const WholeBodySingleton&) = delete;
    WholeBodySingleton& operator=(const WholeBodySingleton&) = delete;

    // UTILITIES
    // =========

    /**
     * Check if a Configuration labelled by confKey exists and is a
     * valid object.
     *
     * @param  confKey The key describing the configuration (name of the Simulink block)
     * @return         True if the configuration is valid
     */
    bool isKeyValid(const std::string& confKey) const;

    /**
     * Returns the DoFs associated with the configuration labelled by confKey
     *
     * @param  confKey The key describing the configuration (name of the Simulink block)
     * @return         The number of degrees of freedom. It returns -1 when failing.
     */
    int numberOfDoFs(const std::string& confKey) const;

    // GET METHODS
    // ===========

    /**
     * Returns the singleton instance to this object.
     *
     * @note The singleton stays in memory until the entire matlab is closed. The WBToolbox
     *       is designed in such a way that assures to reach a clean state when all blocks
     *       have been terminated.
     *
     * @return the singleton instance
     */
    static wbt::base::WholeBodySingleton& sharedInstance();

    /**
     * Returns the Configuration object labelled by confKey.
     * This object is contained into the wbt::RobotInterface object.
     *
     * @see WholeBodySingleton::getRobotInterface
     * @param  confKey The key describing the configuration (name of the Simulink block)
     * @return         A constant reference to the Configuration object
     */
    const Configuration& getConfiguration(const std::string& confKey) const;

    /**
     * Returns a \c shared_ptr to the RobotInterface object containing the configuration
     * labelled by confKey and all the objects used to gather and set robot data
     *
     * @param confKey The key describing the configuration (name of the Simulink block)
     * @return        A \c shared_ptr to the RobotInterface of the requested configuration
     */
    const std::shared_ptr<RobotInterface> getRobotInterface(const std::string& confKey) const;

    /**
     * Returns a \c shared_ptr to the KinDynComputations object used to perform calculation
     * on the provided model
     *
     * @param confKey The key describing the configuration (name of the Simulink block)
     * @return        A \c shared_ptr to the iDynTree::KinDynComputations of the requested
     *                configuration
     */
    const std::shared_ptr<iDynTree::KinDynComputations>
    getKinDynComputations(const std::string& confKey) const;

    // WHOLEBODYSINGLETON CONFIGURATION
    // ================================

    /** Stores in the singleton the new configuration into a RobotInterface object
     *
     * If the config is valid and hasn't been already stored, it creates a new entry
     * in WholeBodySingleton::m_interfaces. If a configuration with matching confKey is found,
     * if the Configuration object is the same it does nothing, otherwise it overrides it.
     *
     * @note Since confKey is the name of the block, the overriding can happen only after
     *       subsequent compilation in Simulink, when the block's parameters have been
     *       changed.
     *
     * @param  config  The wbt::Configuration object parsed from Simulink's parameters
     * @return         Returns a shared pointer to the RobotInterface object created from
     *                 the Configuration object.
     * @see            WholeBodySingleton::isKeyValid
     */
    std::shared_ptr<RobotInterface> createRobotInterface(const Configuration& config);

    /** Stores a new whole-body configuration and return a RobotInterface object
     *
     * @param parameters A wbt::Parameters object containing all the parameters to fill
     *                   a wbt::Configuration object
     * @return Returns a shared pointer to the RobotInterface object created from
     *         the Configuration object.
     * @see    WholeBodySingleton::storeConfiguration, wbt::Configuration,
     *         Parameters::containConfigurationData
     */
    std::shared_ptr<RobotInterface>
    storeConfiguration(const blockfactory::core::Parameters& parameters);

    /**
     * Delete the wbt::RobotInterface referred by confKey. No-op if it doesn't exist.
     *
     * @param confKey The key describing the configuration (name of the Simulink block)
     */
    void eraseConfiguration(const std::string& confKey);
};

#endif // WBT_WHOLEBODYSINGLETON_H
