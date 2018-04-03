#ifndef WBT_DOTJDOTQ_H
#define WBT_DOTJDOTQ_H

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
    bool m_frameIsCoM;
    iDynTree::FrameIndex m_frameIndex;


public:
    static const std::string ClassName;

    DotJNu();
    ~DotJNu() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* WBT_DOTJDOTQ_H */
