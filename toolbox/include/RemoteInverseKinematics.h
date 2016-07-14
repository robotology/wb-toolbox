#ifndef REMOTEINVERSEKINEMATICS_H
#define REMOTEINVERSEKINEMATICS_H

#include "WBIModelBlock.h"

namespace wbt {
    class RemoteInverseKinematics;
}

class wbt::RemoteInverseKinematics : public wbt::Block {

    struct RemoteInverseKinematicsPimpl;
    RemoteInverseKinematicsPimpl *m_piml;

public:
    static std::string ClassName;
    RemoteInverseKinematics();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
    
};


#endif /* end of include guard: REMOTEINVERSEKINEMATICS_H */
