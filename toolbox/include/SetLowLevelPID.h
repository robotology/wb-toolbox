/*

 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_SETLOWLEVELPID_H
#define WBT_SETLOWLEVELPID_H

#include "WBBlock.h"
#include <unordered_map>
#include <yarp/dev/IPidControl.h>

namespace wbt {
    class SetLowLevelPID;
    using PidData = std::tuple<double, double, double>;
} // namespace wbt

namespace yarp {
    namespace dev {
        class Pid;
    }
} // namespace yarp

/**
 * @brief The wbt::SetLowLevelPID class
 *
 * @section Parameters
 *
 * In addition to @ref wbblock_parameters, wbt::SetLowLevelPID requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRUCT_CELL_DOUBLE | 0 + WBBlock::NumberOfParameters | 1 | 1 | "P"           |
 * | ::STRUCT_CELL_DOUBLE | 0 + WBBlock::NumberOfParameters | 1 | 1 | "I"           |
 * | ::STRUCT_CELL_DOUBLE | 0 + WBBlock::NumberOfParameters | 1 | 1 | "D"           |
 * | ::STRING             | 1 + WBBlock::NumberOfParameters | 1 | 1 | "ControlType" |
 *
 */
class wbt::SetLowLevelPID : public wbt::WBBlock
{
private:
    std::vector<yarp::dev::Pid> m_appliedPidValues;
    std::vector<yarp::dev::Pid> m_defaultPidValues;
    std::unordered_map<std::string, PidData> m_pidJointsFromParameters;
    yarp::dev::PidControlTypeEnum m_controlType;

public:
    static const std::string ClassName;

    SetLowLevelPID() = default;
    ~SetLowLevelPID() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_SETLOWLEVELPID_H
