/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_GETMEASUREMENT_H
#define WBT_GETMEASUREMENT_H

#include "WBBlock.h"

namespace wbt {
    class GetMeasurement;
    enum MeasuredType
    {
        MEASUREMENT_JOINT_POS,
        MEASUREMENT_JOINT_VEL,
        MEASUREMENT_JOINT_ACC,
        ESTIMATE_JOINT_TORQUE
    };
} // namespace wbt

/**
 * @brief The wbt::GetMeasurement class
 *
 * @section Parameters
 *
 * In addition to @ref wbblock_parameters, wbt::GetMeasurement requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRING | 0 + WBBlock::NumberOfParameters | 1 | 1 | "MeasuredType" |
 *
 */
class wbt::GetMeasurement : public wbt::WBBlock
{
private:
    std::vector<double> m_measurement;
    wbt::MeasuredType m_measuredType;

public:
    static const std::string ClassName;

    GetMeasurement() = default;
    ~GetMeasurement() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_GETMEASUREMENT_H
