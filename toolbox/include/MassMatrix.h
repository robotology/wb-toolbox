#ifndef WBT_MASSMATRIX_H
#define WBT_MASSMATRIX_H

#include "WBBlock.h"
#include <memory>

namespace wbt {
    class MassMatrix;
}

namespace iDynTree {
    class MatrixDynSize;
}

class wbt::MassMatrix : public wbt::WBBlock
{
private:
    std::unique_ptr<iDynTree::MatrixDynSize> m_massMatrix;

public:
    static const std::string ClassName;
    MassMatrix();
    ~MassMatrix() override = default;

    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* WBT_MASSMATRIX_H */
