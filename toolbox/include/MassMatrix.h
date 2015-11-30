#ifndef WBIT_MASSMATRIX_H
#define WBIT_MASSMATRIX_H

#include "WBIBlock.h"

namespace wbit {
    class MassMatrix;
}

class wbit::MassMatrix : public wbit::WBIBlock {

    double *m_basePose;
    double *m_massMatrix;

    //input buffers
    double *m_basePoseRaw;
    double *m_configuration;

public:
    static std::string ClassName;
    MassMatrix();

    virtual bool configureSizeAndPorts(SimStruct *S, wbit::Error *error);

    virtual bool initialize(SimStruct *S, wbit::Error *error);
    virtual bool terminate(SimStruct *S, wbit::Error *error);
    virtual bool output(SimStruct *S, wbit::Error *error);


};


#endif /* end of include guard: WBIT_MASSMATRIX_H */
