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

    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);

};


#endif /* end of include guard: WBT_YARP2DOFS_H */
