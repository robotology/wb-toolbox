#include "WBIBlock.h"

#include "BlockInformation.h"
#include "Error.h"
#include "WBInterface.h"
#include <yarp/os/Network.h>

#define PARAM_IDX_1 1                           // robot name
#define PARAM_IDX_2 2                           // local (module) name
#define PARAM_IDX_3 3                           // wbi config file
#define PARAM_IDX_4 4                           // wbi list

wbt::WBIBlock::WBIBlock()
: m_wbiConfigurationFileName("")
, m_wbiListName("") {}

wbt::WBIBlock::~WBIBlock() { }

unsigned wbt::WBIBlock::numberOfParameters() { return 4; }

bool wbt::WBIBlock::configureWBIParameters(BlockInformation *blockInfo, wbt::Error *error)
{
    //parameters needed by this block:
    // - YARP_ROBOT_NAME: needed by resource finder for resource lookup (for now it is taken by the environment)
    // - robot: robot port. If defined overrides the one specified by wbi file
    // - moduleName: local (opened) ports.
    // - wbi config file name (default: yarpWholeBodyInterface.ini): specifies the wbi config file
    // - wbi list (default ROBOT_TORQUE_CONTROL_JOINTS): specifies the WBI list .
    //            It it is a normal string, it is interpreted as a named list of joints that is loaded from the
    //            yarpWholeBodyInterface.ini file. If instead it s


    //robot name
    std::string robotName;
    if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_1, robotName)) {
        if (error) error->message = "Cannot retrieve string from robot parameter";
        return false;
    }

    //local name
    std::string localName;
    if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_2, localName)) {
        if (error) error->message = "Cannot retrieve string from localName parameter";
        return false;
    }
    //default local name
    if (localName.empty()) localName = "WBIT";

    //wbi config file
    if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_3, m_wbiConfigurationFileName)) {
        if (error) error->message = "Cannot retrieve string from WBI config file parameter";
        return false;
    }

    //default config file:
    if (m_wbiConfigurationFileName.empty()) m_wbiConfigurationFileName = "yarpWholeBodyInterface.ini";

    //wbi list name
    if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_4, m_wbiListName)) {
        if (error) error->message = "Cannot retrieve string from WBI list parameter";
        return false;
    }

    //default list:
    if (m_wbiListName.empty()) m_wbiListName = "ROBOT_TORQUE_CONTROL_JOINTS";

    WBInterface &interface = WBInterface::sharedInstance();

    Error interfaceError;
    if (!interface.configure(robotName, localName, m_wbiConfigurationFileName, m_wbiListName, &interfaceError)) {
        if (error) error->message = "Failed to configure WholeBodyInterface with error " + interfaceError.message;
        return false;
    }
    return true;
}

bool wbt::WBIBlock::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
{
    //wbi config file
    if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_3, m_wbiConfigurationFileName)) {
        if (error) error->message = "Could not read WBI configuration file parameter";
        return false;
    }

    //default config file:
    if (m_wbiConfigurationFileName.empty()) m_wbiConfigurationFileName = "yarpWholeBodyInterface.ini";

    //wbi list name
    if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_4, m_wbiListName)) {
        if (error) error->message = "Could not read WBI list parameter";
        return false;
    }

    //default list:
    if (m_wbiListName.empty()) m_wbiListName = "ROBOT_TORQUE_CONTROL_JOINTS";

    WBInterface &interface = WBInterface::sharedInstance();

    int dofs = interface.dofsForConfigurationFileAndList(m_wbiConfigurationFileName, m_wbiListName);
    if (dofs == -1) {
        if (error) error->message = "Failed to configure WholeBodyInterface. Could not load WBI properties from file";
        return false;
    } else if (dofs == -2) {
        if (error) error->message = "Failed to configure WholeBodyInterface. WBI joint list not found or failed to configure. Check if joint list exists.";
        return false;
    }

    return true;
}

bool wbt::WBIBlock::initialize(BlockInformation *blockInfo, wbt::Error *error)
{
    using namespace yarp::os;
    Network::init();
    if (!Network::initialized() || !Network::checkNetwork(5.0)) {
        if (error) error->message = "YARP server wasn't found active";
        return false;
    }

    if (!configureWBIParameters(blockInfo, error)) {
        return false;
    }

    WBInterface &interface = WBInterface::sharedInstance();
    if (!interface.initialize()) {
        if (error) error->message = "Failed to initialize WBI";
        return false;
    }
    return true;
}

bool wbt::WBIBlock::terminate(BlockInformation */*S*/, wbt::Error *error)
{
    if (!WBInterface::sharedInstance().terminate()) {
        if (error) error->message = "Failed to terminate WBI";
        return false;
    }
    yarp::os::Network::fini();
    return true;
}
