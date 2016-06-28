#ifndef WBT_MASSMATRIX_H
#define WBT_MASSMATRIX_H

#include "WBIModelBlock.h"

namespace wbt {
    class MassMatrix;
}

class wbt::MassMatrix : public wbt::WBIModelBlock {

    double *m_basePose;
    double *m_massMatrix;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;

public:
    static std::string ClassName;
    MassMatrix();

    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);

    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);


};


#endif /* end of include guard: WBT_MASSMATRIX_H */
