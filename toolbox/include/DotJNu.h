#ifndef WBT_DOTJNU_H
#define WBT_DOTJNU_H

#include "WBBlock.h"
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Model/Indices.h>
#include <memory>

namespace wbt {
    class DotJNu;
}

class wbt::DotJNu : public wbt::WBBlock
{
private:
    // Output
    std::unique_ptr<iDynTree::Vector6> m_dotJNu;

    // Other variables
    bool m_frameIsCoM = false;
    iDynTree::FrameIndex m_frameIndex = iDynTree::FRAME_INVALID_INDEX;

public:
    static const std::string ClassName;

    DotJNu() = default;
    ~DotJNu() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_DOTJNU_H
