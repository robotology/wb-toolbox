#ifndef REMOTEINVERSEKINEMATICS_H
#define REMOTEINVERSEKINEMATICS_H

#include "WBIModelBlock.h"

namespace wbt {
    class RemoteInverseKinematics;
    class BlockInformation;
}

class wbt::RemoteInverseKinematics : public wbt::Block {

    struct RemoteInverseKinematicsPimpl;
    RemoteInverseKinematicsPimpl *m_piml;

public:
    static std::string ClassName;
    RemoteInverseKinematics();

    virtual unsigned numberOfParameters();
    virtual unsigned numberOfDiscreteStates();
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);

    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);
    
    
};


#endif /* end of include guard: REMOTEINVERSEKINEMATICS_H */
