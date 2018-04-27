/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_CENTROIDALMOMENTUM_H
#define WBT_CENTROIDALMOMENTUM_H

#include "WBBlock.h"
#include <memory>

namespace wbt {
    class CentroidalMomentum;
}

/**
 * @brief The wbt::CentroidalMomentum class
 */
class wbt::CentroidalMomentum final : public wbt::WBBlock
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    static const std::string ClassName;

    CentroidalMomentum();
    ~CentroidalMomentum() override = default;

    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_CENTROIDALMOMENTUM_H
