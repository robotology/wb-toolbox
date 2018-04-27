/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_JACOBIAN_H
#define WBT_JACOBIAN_H

#include "WBBlock.h"

#include <memory>
#include <string>

namespace wbt {
    class BlockInformation;
    class Jacobian;
} // namespace wbt

/**
 * @brief The wbt::Jacobian class
 *
 * @section Parameters
 *
 * In addition to @ref wbblock_parameters, wbt::Jacobian requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRING | 0 + WBBlock::NumberOfParameters | 1 | 1 | "Frame" |
 *
 */
class wbt::Jacobian final : public wbt::WBBlock
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    static const std::string ClassName;

    Jacobian();
    ~Jacobian() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_JACOBIAN_H
