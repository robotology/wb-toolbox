#include "WBIBlock.h"

#include "Error.h"
#include "WBInterface.h"
#include <yarp/os/Network.h>

#define PARAM_IDX_1 1                           // robot name
#define PARAM_IDX_2 2                           // local (module) name
#define PARAM_IDX_3 3                           // wbi config file
#define PARAM_IDX_4 4                           // wbi list

wbit::WBIBlock::~WBIBlock() { }

unsigned wbit::WBIBlock::numberOfParameters() { return 4; }

bool wbit::WBIBlock::configureSizeAndPorts(SimStruct *S, wbit::Error *error)
{
    int_T buflen, status;
    char *buffer = NULL;

    //wbi config file
    buflen = (1 + mxGetN(ssGetSFcnParam(S, PARAM_IDX_3))) * sizeof(mxChar);
    buffer = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, PARAM_IDX_3)), buffer, buflen);
    if (status) {
        if (error) error->message = "Cannot retrieve string from WBI config file parameter";
        return false;
    }
    std::string wbiConfigFile = buffer;
    mxFree(buffer); buffer = NULL;
    //default config file:
    if (wbiConfigFile.empty()) wbiConfigFile = "yarpWholeBodyInterface.ini";

    //wbi list name
    buflen = (1 + mxGetN(ssGetSFcnParam(S, PARAM_IDX_4))) * sizeof(mxChar);
    buffer = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, PARAM_IDX_4)), buffer, buflen);
    if (status) {
        if (error) error->message = "Cannot retrieve string from WBI list parameter";
        return false;
    }
    std::string wbiList = buffer;
    mxFree(buffer); buffer = NULL;
    //default list:
    if (wbiList.empty()) wbiList = "ROBOT_TORQUE_CONTROL_JOINTS";

    WBInterface &interface = WBInterface::sharedInstance();

    if (interface.dofsForConfigurationFileAndList(wbiConfigFile, wbiList) < 0) {
        if (error) error->message = "Failed to configure WholeBodyInterface with error ";
        return false;
    }
    
    return true;
}

bool wbit::WBIBlock::initialize(SimStruct *S, wbit::Error *error)
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

    int_T buflen, status;
    char *buffer = NULL;

    //robot name
    buflen = (1 + mxGetN(ssGetSFcnParam(S, PARAM_IDX_1))) * sizeof(mxChar);
    buffer = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, PARAM_IDX_1)), buffer, buflen);
    if (status) {
        if (error) error->message = "Cannot retrieve string from robot parameter";
        return false;
    }
    std::string robotName = buffer;
    mxFree(buffer); buffer = NULL;

    //local name
    buflen = (1 + mxGetN(ssGetSFcnParam(S, PARAM_IDX_2))) * sizeof(mxChar);
    buffer = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, PARAM_IDX_2)), buffer, buflen);
    if (status) {
        if (error) error->message = "Cannot retrieve string from localName parameter";
        return false;
    }
    std::string localName = buffer;
    mxFree(buffer); buffer = NULL;
    //default local name
    if (localName.empty()) localName = "WBIT";

    //wbi config file
    buflen = (1 + mxGetN(ssGetSFcnParam(S, PARAM_IDX_3))) * sizeof(mxChar);
    buffer = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, PARAM_IDX_3)), buffer, buflen);
    if (status) {
        if (error) error->message = "Cannot retrieve string from WBI config file parameter";
        return false;
    }
    std::string wbiConfigFile = buffer;
    mxFree(buffer); buffer = NULL;
    //default config file:
    if (wbiConfigFile.empty()) wbiConfigFile = "yarpWholeBodyInterface.ini";

    //wbi list name
    buflen = (1 + mxGetN(ssGetSFcnParam(S, PARAM_IDX_4))) * sizeof(mxChar);
    buffer = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, PARAM_IDX_4)), buffer, buflen);
    if (status) {
        if (error) error->message = "Cannot retrieve string from WBI list parameter";
        return false;
    }
    std::string wbiList = buffer;
    mxFree(buffer); buffer = NULL;
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

bool wbit::WBIBlock::terminate(SimStruct *S, wbit::Error *error)
{
    if (!WBInterface::sharedInstance().terminate()) {
        if (error) error->message = "Failed to terminate WBI";
        return false;
    }
    yarp::os::Network::fini();
    return true;
}
