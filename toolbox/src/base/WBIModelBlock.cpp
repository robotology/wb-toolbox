#include "WBIModelBlock.h"

wbt::WBIModelBlock::~WBIModelBlock() {}

bool wbt::WBIModelBlock::initialize(SimStruct *S, wbt::Error *error)
{
    return configureWBIParameters(S, error);
}

bool wbt::WBIModelBlock::terminate(SimStruct *S, wbt::Error *error)
{
    return true;
}
