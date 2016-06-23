#ifndef WBT_INVERSEKINEMATICS_H
#define WBT_INVERSEKINEMATICS_H

#include "WBIModelBlock.h"

namespace wbt {
    class InverseKinematics;
}

class wbt::InverseKinematics : public wbt::WBIModelBlock {

    struct InverseKinematicsPimpl;
    InverseKinematicsPimpl *m_piml;

public:
    static std::string ClassName;
    InverseKinematics();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
    
};


#endif /* end of include guard: WBT_INVERSEKINEMATICS_H */
