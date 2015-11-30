#ifndef WBIT_GETBIASFORCES_H
#define WBIT_GETBIASFORCES_H

#include "WBIBlock.h"

namespace wbit {
    class GetBiasForces;
}

class wbit::GetBiasForces : public wbit::WBIBlock {

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

    virtual bool configureSizeAndPorts(SimStruct *S, wbit::Error *error);

    virtual bool initialize(SimStruct *S, wbit::Error *error);
    virtual bool terminate(SimStruct *S, wbit::Error *error);
    virtual bool output(SimStruct *S, wbit::Error *error);
    
    
};


#endif /* end of include guard: WBIT_GETBIASFORCES_H */
