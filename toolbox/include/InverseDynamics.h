#ifndef WBT_INVERSEDYNAMICS_H
#define WBT_INVERSEDYNAMICS_H

#include "WBBlock.h"
#include <iDynTree/Core/VectorFixSize.h>

namespace wbt {
    class InverseDynamics;
}

namespace iDynTree {
    class VectorDynSize;
    class FreeFloatingGeneralizedTorques;
}

class wbt::InverseDynamics : public wbt::WBBlock
{
private:
    iDynTree::Vector6*       m_baseAcceleration;
    iDynTree::VectorDynSize* m_jointsAcceleration;

    // Output
    iDynTree::FreeFloatingGeneralizedTorques* m_torques;

    static const unsigned INPUT_IDX_BASE_POSE;
    static const unsigned INPUT_IDX_JOINTCONF;
    static const unsigned INPUT_IDX_BASE_VEL;
    static const unsigned INPUT_IDX_JOINT_VEL;
    static const unsigned INPUT_IDX_BASE_ACC;
    static const unsigned INPUT_IDX_JOINT_ACC;
    static const unsigned OUTPUT_IDX_TORQUES;

public:
    static const std::string ClassName;
    InverseDynamics();

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(BlockInformation* blockInfo) override;
    bool output(BlockInformation* blockInfo) override;

};

#endif /* WBT_INVERSEDYNAMICS_H */
