#ifndef WBT_DOTJDOTQ_H
#define WBT_DOTJDOTQ_H

#include "WBBlock.h"
#include <memory>
#include <iDynTree/Model/Indices.h>
#include <iDynTree/Core/VectorFixSize.h>

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

    static const unsigned INPUT_IDX_BASE_POSE;
    static const unsigned INPUT_IDX_JOINTCONF;
    static const unsigned INPUT_IDX_BASE_VEL;
    static const unsigned INPUT_IDX_JOINT_VEL;
    static const unsigned OUTPUT_IDX_DOTJ_NU;

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
