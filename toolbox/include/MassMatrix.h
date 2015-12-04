#ifndef WBT_MASSMATRIX_H
#define WBT_MASSMATRIX_H

#include "WBIBlock.h"

namespace wbt {
    class MassMatrix;
}

class wbt::MassMatrix : public wbt::WBIBlock {

    double *m_basePose;
    double *m_massMatrix;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;

public:
    static std::string ClassName;
    MassMatrix();

    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);


};


#endif /* end of include guard: WBT_MASSMATRIX_H */
