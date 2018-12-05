/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_CENTROIDALMOMENTUM_H
#define WBT_CENTROIDALMOMENTUM_H

#include "Base/WBBlock.h"

#include <memory>
#include <string>

namespace blockfactory {
    namespace core {
        class BlockInformation;
    } // namespace core
} // namespace blockfactory

namespace wbt {
    class CentroidalMomentum;
} // namespace wbt

/**
 * @brief The wbt::CentroidalMomentum class
 */
class wbt::CentroidalMomentum final : public wbt::WBBlock
{
private:
    class impl;
    std::unique_ptr<impl> pImpl;

public:
    CentroidalMomentum();
    ~CentroidalMomentum() override;

    bool configureSizeAndPorts(blockfactory::core::BlockInformation* blockInfo) override;
    bool initialize(blockfactory::core::BlockInformation* blockInfo) override;
    bool terminate(const blockfactory::core::BlockInformation* blockInfo) override;
    bool output(const blockfactory::core::BlockInformation* blockInfo) override;
};

#endif // WBT_CENTROIDALMOMENTUM_H
