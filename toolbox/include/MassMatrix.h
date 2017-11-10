#ifndef WBT_MASSMATRIX_H
#define WBT_MASSMATRIX_H

#include "WBBlock.h"

namespace wbt {
    class MassMatrix;
}

namespace iDynTree {
    class MatrixDynSize;
}

class wbt::MassMatrix : public wbt::WBBlock
{
private:
    iDynTree::MatrixDynSize* m_massMatrix;

    static const unsigned INPUT_IDX_BASE_POSE;
    static const unsigned INPUT_IDX_JOINTCONF;
    static const unsigned OUTPUT_IDX_MASS_MAT;

public:
    static const std::string ClassName;
    MassMatrix();
    ~MassMatrix() = default;

    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* WBT_MASSMATRIX_H */
