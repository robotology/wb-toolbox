#include "WBIModelBlock.h"

#include "WBInterface.h"
#include "Error.h"

wbt::WBIModelBlock::~WBIModelBlock() {}

bool wbt::WBIModelBlock::initialize(SimStruct *S, wbt::Error *error)
{
    if (!configureWBIParameters(S, error))
        return false;

    if (!WBInterface::sharedInstance().initializeModel()) {
        if (error) error->message = "Failed to initialize WBI";
        return false;
    }
    return true;
}

bool wbt::WBIModelBlock::terminate(SimStruct *S, wbt::Error *error)
{
    if (!WBInterface::sharedInstance().terminateModel()) {
        if (error) error->message = "Failed to terminate WBI";
        return false;
    }
    return true;
}
