#ifndef WBT_JACOBIAN_H
#define WBT_JACOBIAN_H

#include "WBBlock.h"
#include <iDynTree/Model/Indices.h>
#include <memory>

namespace wbt {
    class Jacobian;
}

namespace iDynTree {
    class MatrixDynSize;
}

class wbt::Jacobian : public wbt::WBBlock
{
private:
    // Support variables
    std::unique_ptr<iDynTree::MatrixDynSize> m_jacobianCOM;

    // Output
    std::unique_ptr<iDynTree::MatrixDynSize> m_jacobian;

    // Other variables
    bool m_frameIsCoM;
    iDynTree::FrameIndex m_frameIndex;

    static const unsigned INPUT_IDX_BASE_POSE;
    static const unsigned INPUT_IDX_JOINTCONF;
    static const unsigned OUTPUT_IDX_FW_FRAME;

public:
    static const std::string ClassName;
    Jacobian();
    ~Jacobian() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* WBT_JACOBIAN_H */
