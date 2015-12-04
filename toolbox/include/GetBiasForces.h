#ifndef WBT_GETBIASFORCES_H
#define WBT_GETBIASFORCES_H

#include "WBIBlock.h"

namespace wbt {
    class GetBiasForces;
}

class wbt::GetBiasForces : public wbt::WBIBlock {

    double *m_basePose;
    double *m_biasForces;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;
    double *m_baseVelocity;
    double *m_jointsVelocity;

public:
    static std::string ClassName;
    GetBiasForces();

    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
    
};


#endif /* end of include guard: WBT_GETBIASFORCES_H */
