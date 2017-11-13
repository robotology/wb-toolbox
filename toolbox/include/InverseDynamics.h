#ifndef WBT_INVERSEDYNAMICS_H
#define WBT_INVERSEDYNAMICS_H

#include "WBBlock.h"
#include <memory>
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
    std::unique_ptr<iDynTree::Vector6>       m_baseAcceleration;
    std::unique_ptr<iDynTree::VectorDynSize> m_jointsAcceleration;

    // Output
    std::unique_ptr<iDynTree::FreeFloatingGeneralizedTorques> m_torques;

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
    ~InverseDynamics() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;

};

#endif /* WBT_INVERSEDYNAMICS_H */
