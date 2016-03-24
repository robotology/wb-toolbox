#ifndef WBT_INVERSEDYNAMICS_H
#define WBT_INVERSEDYNAMICS_H

#include "WBIModelBlock.h"

namespace wbt {
    class InverseDynamics;
}

class wbt::InverseDynamics : public wbt::WBIModelBlock {

    double *m_basePose;
    double *m_torques;

    bool m_explicitGravity;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;
    double *m_baseVelocity;
    double *m_jointsVelocity;
    double *m_baseAcceleration;
    double *m_jointsAcceleration;
    double m_gravity[3];

public:
    static std::string ClassName;
    InverseDynamics();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
    
};


#endif /* end of include guard: WBT_INVERSEDYNAMICS_H */
