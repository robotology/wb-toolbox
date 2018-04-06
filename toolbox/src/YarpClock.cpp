/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "YarpClock.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Signal.h"

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

using namespace wbt;

const std::string YarpClock::ClassName = "YarpClock";

unsigned YarpClock::numberOfParameters()
{
    return Block::numberOfParameters();
}

bool YarpClock::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // INPUTS
    // ======
    //
    // No inputs
    //

    if (!blockInfo->setNumberOfInputPorts(0)) {
        wbtError << "Failed to set input port number to 0.";
        return false;
    }

    // OUTPUT
    // ======
    //
    // 1) The yarp time. In short, it streams yarp::os::Time::now().
    //

    if (!blockInfo->setNumberOfOutputPorts(1)) {
        wbtError << "Failed to set output port number.";
        return false;
    }

    blockInfo->setOutputPortVectorSize(0, 1);
    blockInfo->setOutputPortType(0, DataType::DOUBLE);

    return true;
}

bool YarpClock::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    if (!parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    yarp::os::Network::init();

    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        wbtError << "YARP server wasn't found active.";
        return false;
    }

    return true;
}

bool YarpClock::terminate(const BlockInformation* /*S*/)
{
    yarp::os::Network::fini();
    return true;
}

bool YarpClock::output(const BlockInformation* blockInfo)
{
    Signal outputSignal = blockInfo->getOutputPortSignal(0);
    if (!outputSignal.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }

    if (!outputSignal.set(0, yarp::os::Time::now())) {
        wbtError << "Failed to write data to the output signal.";
        return false;
    }
    return true;
}
