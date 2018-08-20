/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "YarpClock.h"
#include "Core/BlockInformation.h"
#include "Core/Log.h"
#include "Core/Signal.h"

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <ostream>
#include <tuple>

using namespace wbt;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum OutputIndex
{
    Clock = 0,
};

// BLOCK CLASS
// ===========

unsigned YarpClock::numberOfParameters()
{
    return Block::numberOfParameters();
}

bool YarpClock::configureSizeAndPorts(BlockInformation* blockInfo)
{

    // INPUTS
    // ======
    //
    // No inputs
    //
    // OUTPUT
    // ======
    //
    // 1) The yarp time. In short, it streams yarp::os::Time::now().
    //

    const bool ok = blockInfo->setIOPortsData({
        {
            // Inputs
        },
        {
            // Outputs
            std::make_tuple(OutputIndex::Clock, std::vector<int>{1}, DataType::DOUBLE),
        },
    });

    if (!ok) {
        wbtError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool YarpClock::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    yarp::os::Network::init();

    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        wbtError << "YARP server wasn't found active.";
        return false;
    }

    return true;
}

bool YarpClock::terminate(const BlockInformation* /*blockInfo*/)
{
    yarp::os::Network::fini();
    return true;
}

bool YarpClock::output(const BlockInformation* blockInfo)
{
    OutputSignalPtr outputSignal = blockInfo->getOutputPortSignal(OutputIndex::Clock);
    if (!outputSignal) {
        wbtError << "Output signal not valid.";
        return false;
    }

    if (!outputSignal->set(0, yarp::os::Time::now())) {
        wbtError << "Failed to write data to the output signal.";
        return false;
    }
    return true;
}
