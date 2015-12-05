#ifndef WBT_CENTROIDALMOMENTUM_H
#define WBT_CENTROIDALMOMENTUM_H

#include "WBIBlock.h"

namespace wbt {
    class CentroidalMomentum;
}

class wbt::CentroidalMomentum : public wbt::WBIBlock {

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

    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
    
};


#endif /* end of include guard: WBT_CENTROIDALMOMENTUM_H */
