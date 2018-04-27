/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_FORWARDKINEMATICS_H
#define WBT_FORWARDKINEMATICS_H

#include "WBBlock.h"
#include <memory>

namespace wbt {
    class ForwardKinematics;
}

/**
 * @brief The wbt::ForwardKinematics class
 *
 * @section Parameters
 *
 * In addition to @ref wbblock_parameters, wbt::ForwardKinematics requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRING | 0 + WBBlock::NumberOfParameters | 1 | 1 | "Frame" |
 *
 */
class wbt::ForwardKinematics final : public wbt::WBBlock
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    static const std::string ClassName;

    ForwardKinematics();
    ~ForwardKinematics() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_FORWARDKINEMATICS_H
