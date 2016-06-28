#ifndef WBT_CENTROIDALMOMENTUM_H
#define WBT_CENTROIDALMOMENTUM_H

#include "WBIModelBlock.h"

namespace wbt {
    class CentroidalMomentum;
}

class wbt::CentroidalMomentum : public wbt::WBIModelBlock {

    double *m_basePose;
    double *m_centroidalMomentum;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;
    double *m_baseVelocity;
    double *m_jointsVelocity;

public:
    static std::string ClassName;
    CentroidalMomentum();

    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);

    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);
    
    
};


#endif /* end of include guard: WBT_CENTROIDALMOMENTUM_H */
