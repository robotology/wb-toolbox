#include "Block.h"
#include "toolbox.h"

wbt::Block::~Block() {}
void wbt::Block::parameterAtIndexIsTunable(unsigned /*index*/, bool &tunable) { tunable = false; }
bool wbt::Block::checkParameters(SimStruct */*S*/, wbt::Error */*error*/) { return true; }

bool wbt::Block::readStringParameterAtIndex(SimStruct *S, unsigned index, std::string &readParameter)
{
    int_T buflen, status;
    char *buffer = NULL;

    //robot name
    buflen = (1 + mxGetN(ssGetSFcnParam(S, index))) * sizeof(mxChar);
    buffer = static_cast<char*>(mxMalloc(buflen));
    status = mxGetString((ssGetSFcnParam(S, index)), buffer, buflen);
    if (status) {
        return false;
    }
    readParameter = buffer;
    mxFree(buffer); buffer = NULL;
    return true;
}
