/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_YARPREAD_H
#define WBT_YARPREAD_H

#include <BlockFactory/Core/Block.h>

#include <memory>
#include <string>

namespace blockfactory {
    namespace core {
        class BlockInformation;
    } // namespace core
} // namespace blockfactory

namespace wbt {
    class YarpRead;
} // namespace wbt

/**
 * @brief The wbt::YarpRead class
 *
 * In addition to @ref block_parameters, wbt::YarpRead requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRING | 0 + Block::NumberOfParameters | 1 | 1 | "PortName"           |
 * | ::INT    | 1 + Block::NumberOfParameters | 1 | 1 | "SignalSize"         |
 * | ::BOOL   | 2 + Block::NumberOfParameters | 1 | 1 | "WaitData"           |
 * | ::BOOL   | 3 + Block::NumberOfParameters | 1 | 1 | "ReadTimestamp"      |
 * | ::BOOL   | 4 + Block::NumberOfParameters | 1 | 1 | "Autoconnect"        |
 * | ::DOUBLE | 5 + Block::NumberOfParameters | 1 | 1 | "Timeout"            |
 * | ::BOOL   | 6 + Block::NumberOfParameters | 1 | 1 | "ErrorOnMissingPort" |
 *
 */
class wbt::YarpRead final : public blockfactory::core::Block
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    YarpRead();
    ~YarpRead() override;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
};

#endif // WBT_YARPREAD_H
