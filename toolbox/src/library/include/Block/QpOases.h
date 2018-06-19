/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "Core/Block.h"

#include <memory>
#include <string>

#ifndef WBT_QPOASES_H
#define WBT_QPOASES_H

namespace wbt {
    class BlockInformation;
    class QpOases;
} // namespace wbt

/**
 * @brief The wbt::QpOases class
 *
 * @section Parameters
 *
 * In addition to @ref block_parameters, wbt::DiscreteFilter requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ParameterType::BOOL | 0 + Block::NumberOfParameters | 1 | 1 | "UseLba"        |
 * | ParameterType::BOOL | 1 + Block::NumberOfParameters | 1 | 1 | "UseUbA"        |
 * | ParameterType::BOOL | 2 + Block::NumberOfParameters | 1 | 1 | "UseLb"         |
 * | ParameterType::BOOL | 3 + Block::NumberOfParameters | 1 | 1 | "UseUb"         |
 * | ParameterType::BOOL | 4 + Block::NumberOfParameters | 1 | 1 | "ComputeObjVal" |
 * | ParameterType::BOOL | 5 + Block::NumberOfParameters | 1 | 1 | "StopWhenFails" |
 *
 */
class wbt::QpOases final : public wbt::Block
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    static const std::string ClassName;

    QpOases();
    ~QpOases() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool initializeInitialConditions(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_QPOASES_H
