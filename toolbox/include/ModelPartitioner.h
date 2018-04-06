/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_MODELPARTITIONER_H
#define WBT_MODELPARTITIONER_H

#include "RobotInterface.h"
#include "WBBlock.h"
#include <memory>
#include <vector>

namespace wbt {
    class ModelPartitioner;
}

class wbt::ModelPartitioner : public wbt::WBBlock
{
private:
    bool m_vectorToControlBoards;

    std::shared_ptr<JointNameToYarpMap> m_jointNameToYarpMap;
    std::shared_ptr<JointNameToIndexInControlBoardMap> m_jointNameToIndexInControlBoardMap;
    std::shared_ptr<ControlBoardIndexLimit> m_controlBoardIndexLimit;

public:
    static const std::string ClassName;

    ModelPartitioner() = default;
    ~ModelPartitioner() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_MODELPARTITIONER_H
