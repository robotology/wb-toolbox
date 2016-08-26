#ifndef WBT_YARP2DOFS_H
#define WBT_YARP2DOFS_H

#include "WBIModelBlock.h"

namespace wbt {
    class YARPWBIConverter;
}

class wbt::YARPWBIConverter : public wbt::WBIModelBlock {

    struct YARPWBIConverterPimpl;
    YARPWBIConverterPimpl *m_pimpl;

public:
    static std::string ClassName;
    YARPWBIConverter();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);


};


#endif /* end of include guard: WBT_YARP2DOFS_H */
