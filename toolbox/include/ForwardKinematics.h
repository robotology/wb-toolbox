#ifndef WBT_FORWARDKINEMATICS_H
#define WBT_FORWARDKINEMATICS_H

#include "WBBlock.h"
#include <iDynTree/Model/Indices.h>
#include <memory>

namespace wbt {
    class ForwardKinematics;
}

class wbt::ForwardKinematics : public wbt::WBBlock
{
private:
    bool m_frameIsCoM;
    iDynTree::FrameIndex m_frameIndex;

public:
    static const std::string ClassName;

    ForwardKinematics();
    ~ForwardKinematics() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* WBT_FORWARDKINEMATICS_H */
