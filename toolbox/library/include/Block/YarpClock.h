/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_YARPCLOCK_H
#define WBT_YARPCLOCK_H

#include <BlockFactory/Core/Block.h>

#include <string>

namespace blockfactory {
    namespace core {
        class BlockInformation;
    } // namespace core
} // namespace blockfactory

namespace wbt {
    class YarpClock;
} // namespace wbt

/**
 * @brief The wbt::YarpClock class
 */
class wbt::YarpClock final : public blockfactory::core::Block
{
public:
    YarpClock() = default;
    ~YarpClock() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
};

#endif // WBT_YARPCLOCK_H
