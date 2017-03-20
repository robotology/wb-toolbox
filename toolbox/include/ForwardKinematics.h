#ifndef WBT_FORWARDKINEMATICS_H
#define WBT_FORWARDKINEMATICS_H

#include "WBIModelBlock.h"

namespace wbt {
    class ForwardKinematics;
}

class wbt::ForwardKinematics : public wbt::WBIModelBlock {

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
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);

    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);


};


#endif /* end of include guard: WBT_FORWARDKINEMATICS_H */
