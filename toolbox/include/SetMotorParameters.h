/*

 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_SETMOTORPARAMETERS_H
#define WBT_SETMOTORPARAMETERS_H

#include "WBBlock.h"

#include <memory>
#include <string>

namespace wbt {
    class BlockInformation;
    class SetMotorParameters;
} // namespace wbt

/**
 * @brief The wbt::SetMotorParameters class
 *
 * @section Parameters
 *
 * In addition to @ref wbblock_parameters, wbt::SetMotorParameters requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::BOOL   | 0 + WBBlock::NumberOfParameters | 1 | 1 | "SetP"        |
 * | ::BOOL   | 1 + WBBlock::NumberOfParameters | 1 | 1 | "SetI"        |
 * | ::BOOL   | 2 + WBBlock::NumberOfParameters | 1 | 1 | "SetD"        |
 * | ::STRING | 3 + WBBlock::NumberOfParameters | 1 | 1 | "ControlType" |
 * | ::BOOL   | 4 + WBBlock::NumberOfParameters | 1 | 1 | "SetKTau"     |
 * | ::STRING | 5 + WBBlock::NumberOfParameters | 1 | ParameterMetadata::DynamicSize | "KTau" |
 * | ::BOOL   | 6 + WBBlock::NumberOfParameters | 1 | 1 | "SetBemf"     |
 * | ::STRING | 7 + WBBlock::NumberOfParameters | 1 | ParameterMetadata::DynamicSize | "Bemf" |
 * *
 */
class wbt::SetMotorParameters final : public wbt::WBBlock
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    static const std::string ClassName;

    SetMotorParameters();
    ~SetMotorParameters() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_SETMOTORPARAMETERS_H
