/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/YarpClock.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Signal.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <ostream>
#include <tuple>

using namespace wbt::block;
using namespace blockfactory::core;

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

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
        },
        {
            // Outputs
            {OutputIndex::Clock, Port::Dimensions{1}, Port::DataType::DOUBLE},
        });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
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
        bfError << "YARP server wasn't found active.";
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
        bfError << "Output signal not valid.";
        return false;
    }

    if (!outputSignal->set(0, yarp::os::Time::now())) {
        bfError << "Failed to write data to the output signal.";
        return false;
    }
    return true;
}
