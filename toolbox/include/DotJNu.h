/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_DOTJNU_H
#define WBT_DOTJNU_H

#include "WBBlock.h"
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/Model/Indices.h>
#include <memory>

namespace wbt {
    class DotJNu;
}

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
class wbt::DotJNu : public wbt::WBBlock
{
private:
    // Output
    std::unique_ptr<iDynTree::Vector6> m_dotJNu;

    // Other variables
    bool m_frameIsCoM = false;
    iDynTree::FrameIndex m_frameIndex = iDynTree::FRAME_INVALID_INDEX;

public:
    static const std::string ClassName;

    DotJNu() = default;
    ~DotJNu() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_DOTJNU_H
