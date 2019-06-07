/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_YARPRPC_H
#define WBT_YARPRPC_H

#include <BlockFactory/Core/Block.h>

#include <string>

namespace wbt {
    namespace block {
        class YarpRpc;
    } // namespace block
} // namespace wbt

namespace blockfactory {
    namespace core {
        class BlockInformation;
    } // namespace core
} // namespace blockfactory

/**
 * @brief The wbt::YarpRpc class
 *
 * In addition to @ref block_parameters, wbt::YarpRpc requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::BOOL   | 0 + Block::NumberOfParameters | 1 | 1 | "RisingEdgeTrigger" |
 * | ::STRING | 1 + Block::NumberOfParameters | 1 | 1 | "PortName"           |
 * | ::STRING | 2 + Block::NumberOfParameters | 1 | 1 | "RpcCommand"         |
 *
 */
class wbt::block::YarpRpc final : public blockfactory::core::Block
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    YarpRpc();
    ~YarpRpc() override;

    unsigned numberOfParameters() override;
    bool parseParameters(blockfactory::core::BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;

    bool sendRpcCommand();
};

#endif // WBT_YARPRPC_H
