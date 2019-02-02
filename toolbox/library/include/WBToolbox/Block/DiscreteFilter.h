/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_FILTER_H
#define WBT_FILTER_H

#include <BlockFactory/Core/Block.h>

#include <memory>
#include <string>

namespace wbt {
    namespace block {
        class DiscreteFilter;
    } // namespace block
} // namespace wbt

namespace blockfactory {
    namespace core {
        class BlockInformation;
    } // namespace core
} // namespace blockfactory

// clang-format off
/**
 * @brief The wbt::DiscreteFilter class
 *
 * @section Parameters
 *
 * In addition to @ref block_parameters, wbt::DiscreteFilter requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "Fc"          |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "Ts"          |
 * | ::STRUCT_INT    | 0 + Block::NumberOfParameters | 1 | 1             | "MedianOrder" |
 * | ::STRUCT_STRING | 0 + Block::NumberOfParameters | 1 | 1             | "FilterType"  |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | ParameterMetadata::DynamicSize | "NumCoeffs"   |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | ParameterMetadata::DynamicSize | "DenCoeffs"   |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "InitStatus"  |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "y0"          |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "u0"          |
 *
 */
// clang-format on
class wbt::block::DiscreteFilter final : public blockfactory::core::Block
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    DiscreteFilter();
    ~DiscreteFilter() override;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool
    initializeInitialConditions(const blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
};

#endif // WBT_FILTER_H
