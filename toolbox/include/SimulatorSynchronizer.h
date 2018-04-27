/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_SIMULATORSYNCHRONIZER_H
#define WBT_SIMULATORSYNCHRONIZER_H

#include "Block.h"
#include <memory>

namespace wbt {
    class SimulatorSynchronizer;
}

/**
 * @brief The wbt::SimulatorSynchronizer class
 *
 * @section Parameters
 *
 * In addition to @ref wbblock_parameters, wbt::SetReferences requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::DOUBLE | 0 + WBBlock::NumberOfParameters | 1 | 1 | "Period"          |
 * | ::STRING | 1 + WBBlock::NumberOfParameters | 1 | 1 | "RpcPort"         |
 * | ::STRING | 2 + WBBlock::NumberOfParameters | 1 | 1 | "GazeboClockPort" |
 *
 */
class wbt::SimulatorSynchronizer : public wbt::Block
{
private:
    double m_period = 0.01;
    bool m_firstRun = true;

    struct RPCData;
    std::unique_ptr<RPCData> m_rpcData;

public:
    static const std::string ClassName;

    SimulatorSynchronizer();
    ~SimulatorSynchronizer() override = default;

    unsigned numberOfParameters() override;
    std::vector<std::string> additionalBlockOptions() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_SIMULATORSYNCHRONIZER_H
