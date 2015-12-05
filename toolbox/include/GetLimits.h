#ifndef WBT_GETLIMITS_H
#define WBT_GETLIMITS_H

#include "WBIBlock.h"

namespace wbt {
    class GetLimits;
}

class wbt::GetLimits : public wbt::WBIBlock {

    struct Limit;
    struct Limit *m_limits;
    
public:
    static std::string ClassName;
    GetLimits();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);


};


#endif /* end of include guard: WBT_GETLIMITS_H */
