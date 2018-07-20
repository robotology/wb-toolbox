/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_REALTIMESYNCHRONIZER_H
#define WBT_REALTIMESYNCHRONIZER_H

#include "Core/Block.h"

#include <memory>
#include <string>

namespace wbt {
    class BlockInformation;
    class RealTimeSynchronizer;
} // namespace wbt

/**
 * @brief The wbt::RealTimeSynchronizer class
 *
 * @section Parameters
 *
 * In addition to @ref block_parameters, wbt::RealTimeSynchronizer requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::DOUBLE | 0 + Block::NumberOfParameters | 1 | 1 | "Period" |
 *
 */
class wbt::RealTimeSynchronizer final : public wbt::Block
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    static const std::string ClassName;

    RealTimeSynchronizer();
    ~RealTimeSynchronizer() override;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_REALTIMESYNCHRONIZER_H
