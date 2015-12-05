#ifndef WBT_DOTJDOTQ_H
#define WBT_DOTJDOTQ_H

#include "WBIBlock.h"

namespace wbt {
    class DotJDotQ;
}

class wbt::DotJDotQ : public wbt::WBIBlock {

    double *m_basePose;
    double *m_dotJDotQ;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;
    double *m_baseVelocity;
    double *m_jointsVelocity;

    int m_frameIndex;

public:
    static std::string ClassName;
    DotJDotQ();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
    
};


#endif /* end of include guard: WBT_DOTJDOTQ_H */
