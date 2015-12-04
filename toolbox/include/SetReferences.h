#ifndef WBIT_SETREFERENCES_H
#define WBIT_SETREFERENCES_H

#include "WBIBlock.h"

namespace wbit {
    class SetReferences;
}

class wbit::SetReferences : public wbit::WBIBlock {

    double *m_references;

public:
    static std::string ClassName;
    SetReferences();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbit::Error *error);

    virtual bool initialize(SimStruct *S, wbit::Error *error);
    virtual bool terminate(SimStruct *S, wbit::Error *error);
    virtual bool output(SimStruct *S, wbit::Error *error);


};


#endif /* end of include guard: WBIT_SETREFERENCES_H */
