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

#include <memory>
#include <string>

namespace wbt {
    class BlockInformation;
    class SetLowLevelPID;
} // namespace wbt

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
class wbt::SetLowLevelPID final : public wbt::WBBlock
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    static const std::string ClassName;

    SetLowLevelPID();
    ~SetLowLevelPID() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_SETLOWLEVELPID_H
