/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/RealTimeSynchronizer.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <ostream>
#include <tuple>

using namespace wbt::block;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = Block::NumberOfParameters - 1,
    Period
};

// BLOCK PIMPL
// ===========

class RealTimeSynchronizer::impl
{
public:
    double period = 0.01;
    double initialTime;
    unsigned long counter = 0;
};

// BLOCK CLASS
// ===========

RealTimeSynchronizer::RealTimeSynchronizer()
    : pImpl{new impl()}
{}

RealTimeSynchronizer::~RealTimeSynchronizer() = default;

unsigned RealTimeSynchronizer::numberOfParameters()
{
    return Block::numberOfParameters() + 1;
}

bool RealTimeSynchronizer::parseParameters(BlockInformation* blockInfo)
{
    const ParameterMetadata periodMetadata(
        ParameterType::DOUBLE, ParamIndex::Period, 1, 1, "Period");

    if (!blockInfo->addParameterMetadata(periodMetadata)) {
        bfError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool RealTimeSynchronizer::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // INPUTS
    // ======
    //
    // No inputs
    //
    // OUTPUTS
    // =======
    //
    // No outputs
    //

    const bool ok = blockInfo->setIOPortsData({{}, {}});

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool RealTimeSynchronizer::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!RealTimeSynchronizer::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    if (!m_parameters.getParameter("Period", pImpl->period)) {
        bfError << "Failed to get parameter 'period' after its parsing.";
        return false;
    }

    if (pImpl->period <= 0) {
        bfError << "Period must be greater than 0.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    yarp::os::Network::init();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        bfError << "YARP server wasn't found active!!";
        return false;
    }

    return true;
}

bool RealTimeSynchronizer::terminate(const BlockInformation* /*blockInfo*/)
{
    yarp::os::Network::fini();
    return true;
}

bool RealTimeSynchronizer::output(const BlockInformation* /*blockInfo*/)
{
    if (pImpl->counter == 0) {
        pImpl->initialTime = yarp::os::Time::now();
    }

    // read current time
    double currentTime = yarp::os::Time::now() - pImpl->initialTime;
    double desiredTime = pImpl->counter * pImpl->period;

    double sleepPeriod = desiredTime - currentTime;

    // sleep for the remaining time
    if (sleepPeriod > 0) {
        yarp::os::Time::delay(sleepPeriod);
    }

    pImpl->counter++;

    return true;
}
