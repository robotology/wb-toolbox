#ifndef WBT_CENTROIDALMOMENTUM_H
#define WBT_CENTROIDALMOMENTUM_H

#include "WBBlock.h"
#include <memory>

namespace wbt {
    class CentroidalMomentum;
}

namespace iDynTree {
    class SpatialMomentum;
}

class wbt::CentroidalMomentum : public wbt::WBBlock
{
private:
    std::unique_ptr<iDynTree::SpatialMomentum> m_centroidalMomentum;

public:
    static const std::string ClassName;

    CentroidalMomentum();
    ~CentroidalMomentum() override = default;

    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* WBT_CENTROIDALMOMENTUM_H */
