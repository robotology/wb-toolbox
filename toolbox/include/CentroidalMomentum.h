#ifndef WBT_CENTROIDALMOMENTUM_H
#define WBT_CENTROIDALMOMENTUM_H

#include "WBBlock.h"

namespace wbt {
    class CentroidalMomentum;
}

namespace iDynTree {
    class SpatialMomentum;
}

class wbt::CentroidalMomentum : public wbt::WBBlock
{
private:
    iDynTree::SpatialMomentum* m_centroidalMomentum;

    static const unsigned INPUT_IDX_BASE_POSE;
    static const unsigned INPUT_IDX_JOINTCONF;
    static const unsigned INPUT_IDX_BASE_VEL;
    static const unsigned INPUT_IDX_JOINT_VEL;
    static const unsigned OUTPUT_IDX_CENTRMOM;

public:
    static const std::string ClassName;
    CentroidalMomentum();

    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(BlockInformation* blockInfo) override;
    bool output(BlockInformation* blockInfo) override;
};

#endif /* WBT_CENTROIDALMOMENTUM_H */
