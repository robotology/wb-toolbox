#ifndef WBT_DOTJDOTQ_H
#define WBT_DOTJDOTQ_H

#include "WBIModelBlock.h"

namespace wbt {
    class DotJNu;
}

class wbt::DotJNu : public wbt::WBIModelBlock {

    double *m_basePose;
    double *m_dotJNu;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;
    double *m_baseVelocity;
    double *m_jointsVelocity;

    int m_frameIndex;

public:
    static std::string ClassName;
    DotJNu();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);

    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);
    
    
};


#endif /* end of include guard: WBT_DOTJDOTQ_H */
