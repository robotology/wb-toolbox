#include "WBIModelBlock.h"

#include "WBInterface.h"
#include "Error.h"

wbt::WBIModelBlock::~WBIModelBlock() {}

bool wbt::WBIModelBlock::initialize(wbt::BlockInformation *blockInfo, wbt::Error *error)
{
    if (!configureWBIParameters(blockInfo, error))
        return false;

    if (!WBInterface::sharedInstance().initializeModel()) {
        if (error) error->message = "Failed to initialize WBI";
        return false;
    }
    return true;
}

bool wbt::WBIModelBlock::terminate(wbt::BlockInformation *blockInfo, wbt::Error *error)
{
    if (!WBInterface::sharedInstance().terminateModel()) {
        if (error) error->message = "Failed to terminate WBI";
        return false;
    }
    return true;
}
