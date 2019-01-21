/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_YARPWRITE_H
#define WBT_YARPWRITE_H

#include <BlockFactory/Core/Block.h>

#include <memory>
#include <string>

namespace wbt {
    namespace block {
        class YarpWrite;
    } // namespace block
} // namespace wbt

namespace blockfactory {
    namespace core {
        class BlockInformation;
    } // namespace core
} // namespace blockfactory

/**
 * @brief The wbt::YarpWrite class
 *
 * In addition to @ref block_parameters, wbt::YarpWrite requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRING | 0 + Block::NumberOfParameters | 1 | 1 | "PortName"           |
 * | ::BOOL   | 1 + Block::NumberOfParameters | 1 | 1 | "Autoconnect"        |
 * | ::BOOL   | 2 + Block::NumberOfParameters | 1 | 1 | "ErrorOnMissingPort" |
 *
 */
class wbt::block::YarpWrite final : public blockfactory::core::Block
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    YarpWrite();
    ~YarpWrite() override;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
};

#endif // WBT_YARPWRITE_H
