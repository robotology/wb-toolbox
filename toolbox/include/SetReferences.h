#ifndef WBT_SETREFERENCES_H
#define WBT_SETREFERENCES_H

#include "WBIBlock.h"

namespace wbt {
    class SetReferences;
}

class wbt::SetReferences : public wbt::WBIBlock {

    double *m_references;

public:
    static std::string ClassName;
    SetReferences();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);


};


#endif /* end of include guard: WBT_SETREFERENCES_H */
