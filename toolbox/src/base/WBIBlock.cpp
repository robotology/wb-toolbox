#include "WBIBlock.h"

#include "Error.h"
#include "WBInterface.h"
#include <yarp/os/Network.h>

#define PARAM_IDX_1 1                           // robot name
#define PARAM_IDX_2 2                           // local (module) name
#define PARAM_IDX_3 3                           // wbi config file
#define PARAM_IDX_4 4                           // wbi list

wbt::WBIBlock::~WBIBlock() { }

unsigned wbt::WBIBlock::numberOfParameters() { return 4; }

bool wbt::WBIBlock::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
{
    //wbi config file
    std::string wbiConfigFile;
    if (!Block::readStringParameterAtIndex(S, PARAM_IDX_3, wbiConfigFile)) {
        if (error) error->message = "Could not read WBI configuration file parameter";
        return false;
    }

    //default config file:
    if (wbiConfigFile.empty()) wbiConfigFile = "yarpWholeBodyInterface.ini";

    //wbi list name
    std::string wbiList;
    if (!Block::readStringParameterAtIndex(S, PARAM_IDX_4, wbiList)) {
        if (error) error->message = "Could not read WBI list parameter";
        return false;
    }

    //default list:
    if (wbiList.empty()) wbiList = "ROBOT_TORQUE_CONTROL_JOINTS";

    WBInterface &interface = WBInterface::sharedInstance();

    if (interface.dofsForConfigurationFileAndList(wbiConfigFile, wbiList) < 0) {
        if (error) error->message = "Failed to configure WholeBodyInterface with error ";
        return false;
    }
    
    return true;
}

bool wbt::WBIBlock::initialize(SimStruct *S, wbt::Error *error)
{
    using namespace yarp::os;
    Network::init();
    if (!Network::initialized() || !Network::checkNetwork(5.0)) {
        if (error) error->message = "YARP server wasn't found active";
        return false;
    }

    //parameters needed by this block:
    // - YARP_ROBOT_NAME: needed by resource finder for resource lookup (for now it is taken by the environment)
    // - robot: robot port. If defined overrides the one specified by wbi file
    // - moduleName: local (opened) ports.
    // - wbi config file name (default: yarpWholeBodyInterface.ini): specifies the wbi config file
    // - wbi list (default ROBOT_TORQUE_CONTROL_JOINTS): specifies the WBI list

    //robot name
    std::string robotName;
    if (!Block::readStringParameterAtIndex(S, PARAM_IDX_1, robotName)) {
        if (error) error->message = "Cannot retrieve string from robot parameter";
        return false;
    }

    //local name
    std::string localName;
    if (!Block::readStringParameterAtIndex(S, PARAM_IDX_2, localName)) {
        if (error) error->message = "Cannot retrieve string from localName parameter";
        return false;
    }
    //default local name
    if (localName.empty()) localName = "WBIT";

    //wbi config file
    std::string wbiConfigFile;
    if (!Block::readStringParameterAtIndex(S, PARAM_IDX_3, wbiConfigFile)) {
        if (error) error->message = "Cannot retrieve string from WBI config file parameter";
        return false;
    }

    //default config file:
    if (wbiConfigFile.empty()) wbiConfigFile = "yarpWholeBodyInterface.ini";

    //wbi list name
    std::string wbiList;
    if (!Block::readStringParameterAtIndex(S, PARAM_IDX_4, wbiList)) {
        if (error) error->message = "Cannot retrieve string from WBI list parameter";
        return false;
    }

    //default list:
    if (wbiList.empty()) wbiList = "ROBOT_TORQUE_CONTROL_JOINTS";

    WBInterface &interface = WBInterface::sharedInstance();

    Error interfaceError;
    if (!interface.configure(robotName, localName, wbiConfigFile, wbiList, &interfaceError)) {
        if (error) error->message = "Failed to configure WholeBodyInterface with error " + interfaceError.message;
        return false;
    }

    if (!interface.initialize()) {
        if (error) error->message = "Failed to initialize WBI";
        return false;
    }
    return true;
}

bool wbt::WBIBlock::terminate(SimStruct */*S*/, wbt::Error *error)
{
    if (!WBInterface::sharedInstance().terminate()) {
        if (error) error->message = "Failed to terminate WBI";
        return false;
    }
    yarp::os::Network::fini();
    return true;
}
