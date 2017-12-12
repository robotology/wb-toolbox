#ifndef WBT_GETMEASUREMENT_H
#define WBT_GETMEASUREMENT_H

#include "WBBlock.h"

namespace wbt {
    class GetMeasurement;
    enum MeasuredType {
        MEASUREMENT_JOINT_POS,
        MEASUREMENT_JOINT_VEL,
        MEASUREMENT_JOINT_ACC,
        ESTIMATE_JOINT_TORQUE
    };
}


class wbt::GetMeasurement : public wbt::WBBlock
{
private:
    std::vector<double> m_measurement;
    wbt::MeasuredType m_measuredType;
    static void deg2rad(std::vector<double>& v);

public:
    static const std::string ClassName;

    GetMeasurement() = default;
    ~GetMeasurement() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* end of include guard: WBT_GETMEASUREMENT_H */
