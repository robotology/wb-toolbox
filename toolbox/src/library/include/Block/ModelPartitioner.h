/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_MODELPARTITIONER_H
#define WBT_MODELPARTITIONER_H

#include "Base/WBBlock.h"

#include <memory>
#include <string>

namespace wbt {
    class BlockInformation;
    class ModelPartitioner;
} // namespace wbt

/**
 * @brief The wbt::ModelPartitioner class
 *
 * @section Parameters
 *
 * In addition to @ref wbblock_parameters, wbt::ModelPartitioner requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::BOOL | 0 + WBBlock::NumberOfParameters | 1 | 1 | "VectorToControlBoards" |
 */
class wbt::ModelPartitioner final : public wbt::WBBlock
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    static const std::string ClassName;

    ModelPartitioner();
    ~ModelPartitioner() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_MODELPARTITIONER_H
