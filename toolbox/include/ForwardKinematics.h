#ifndef WBIT_FORWARDKINEMATICS_H
#define WBIT_FORWARDKINEMATICS_H

#include "WBIBlock.h"

namespace wbit {
    class ForwardKinematics;
}

class wbit::ForwardKinematics : public wbit::WBIBlock {

    double *m_basePose;
    double *m_frameForwardKinematics;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;

    int m_frameIndex;

public:
    static std::string ClassName;
    ForwardKinematics();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbit::Error *error);

    virtual bool initialize(SimStruct *S, wbit::Error *error);
    virtual bool terminate(SimStruct *S, wbit::Error *error);
    virtual bool output(SimStruct *S, wbit::Error *error);


};


#endif /* end of include guard: WBIT_FORWARDKINEMATICS_H */
