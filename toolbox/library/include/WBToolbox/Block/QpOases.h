/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_QPOASES_H
#define WBT_QPOASES_H

#include <BlockFactory/Core/Block.h>

#include <memory>
#include <string>

namespace wbt {
    namespace block {
        class QpOases;
    } // namespace block
} // namespace wbt

namespace blockfactory {
    namespace core {
        class BlockInformation;
    } // namespace core
} // namespace blockfactory

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
class wbt::block::QpOases final : public blockfactory::core::Block
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    QpOases();
    ~QpOases() override;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool
    initializeInitialConditions(const blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
};

#endif // WBT_QPOASES_H
