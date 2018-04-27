/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_YARPWRITE_H
#define WBT_YARPWRITE_H

#include "Block.h"
#include <memory>

namespace wbt {
    class YarpWrite;
}

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
class wbt::YarpWrite final : public wbt::Block
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    static const std::string ClassName;

    YarpWrite();
    ~YarpWrite() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_YARPWRITE_H
