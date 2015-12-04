#ifndef WBT_JACOBIAN_H
#define WBT_JACOBIAN_H

#include "WBIBlock.h"

namespace wbt {
    class Jacobian;
}

class wbt::Jacobian : public wbt::WBIBlock {

    double *m_basePose;
    double *m_jacobian;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;

    int m_frameIndex;

public:
    static std::string ClassName;
    Jacobian();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);


};


#endif /* end of include guard: WBT_JACOBIAN_H */
