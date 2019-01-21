#ifndef REMOTEINVERSEKINEMATICS_H
#define REMOTEINVERSEKINEMATICS_H

#include "WBIModelBlock.h"

namespace wbt {
    class RemoteInverseKinematics;
    class BlockInformation;
} // namespace wbt

class wbt::RemoteInverseKinematics : public blockfactory::core::Block
{

    struct RemoteInverseKinematicsPimpl;
    RemoteInverseKinematicsPimpl* m_piml;

public:
    static std::string ClassName;
    RemoteInverseKinematics();

    virtual unsigned numberOfParameters();
    virtual unsigned numberOfDiscreteStates();
    virtual bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo, wbt::Error* error);

    virtual bool initialize(blockfactory::core::BlockInformation* blockInfo, wbt::Error* error);
    virtual bool terminate(BlockInformation* blockInfo, wbt::Error* error);
    virtual bool output(BlockInformation* blockInfo, wbt::Error* error);
};

#endif /* end of include guard: REMOTEINVERSEKINEMATICS_H */
