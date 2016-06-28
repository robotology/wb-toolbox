#ifndef WBT_GETESTIMATE_H
#define WBT_GETESTIMATE_H

#include "WBIBlock.h"
#include <wbi/wbiConstants.h>

namespace wbt {
    class GetEstimate;
}

class wbt::GetEstimate : public wbt::WBIBlock {

    double *m_estimate;
    wbi::EstimateType m_estimateType;
    
public:
    static std::string ClassName;
    GetEstimate();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);

    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);


};


#endif /* end of include guard: WBT_GETESTIMATE_H */
