/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_DOTJNU_H
#define WBT_DOTJNU_H

#include "Base/WBBlock.h"

#include <memory>
#include <string>

namespace wbt {
    class BlockInformation;
    class DotJNu;
} // namespace wbt

/**
 * @brief The wbt::DotJNu class
 *
 * @section Parameters
 *
 * In addition to @ref wbblock_parameters, wbt::DotJNu requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRING | 0 + WBBlock::NumberOfParameters | 1 | 1 | "Frame" |
 *
 */
class wbt::DotJNu final : public wbt::WBBlock
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    DotJNu();
    ~DotJNu() override;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_DOTJNU_H
