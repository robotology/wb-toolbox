#ifndef WBT_GETMEASUREMENT_H
#define WBT_GETMEASUREMENT_H

#include "WBBlock.h"

namespace wbt {
    class GetMeasurement;
    enum EstimateType {
        ESTIMATE_JOINT_POS,
        ESTIMATE_JOINT_VEL,
        ESTIMATE_JOINT_ACC,
        ESTIMATE_JOINT_TORQUE
    };
}


class wbt::GetMeasurement : public wbt::WBBlock
{
private:
    std::vector<double> m_estimate;
    wbt::EstimateType m_estimateType;
    static void deg2rad(std::vector<double>& v);

public:
    static const std::string ClassName;

    GetMeasurement() = default;
    ~GetMeasurement() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(BlockInformation* blockInfo) override;
    bool output(BlockInformation* blockInfo) override;
};

#endif /* end of include guard: WBT_GETMEASUREMENT_H */
