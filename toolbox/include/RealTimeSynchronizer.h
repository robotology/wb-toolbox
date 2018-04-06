/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_REALTIMESYNCHRONIZER_H
#define WBT_REALTIMESYNCHRONIZER_H

#include "Block.h"

namespace wbt {
    class RealTimeSynchronizer;
}

class wbt::RealTimeSynchronizer : public wbt::Block
{
private:
    double m_period = 0.01;
    double m_initialTime;
    unsigned long m_counter;

public:
    static const std::string ClassName;

    RealTimeSynchronizer() = default;
    ~RealTimeSynchronizer() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_REALTIMESYNCHRONIZER_H
