/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "RealTimeSynchronizer.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

#include <ostream>

using namespace wbt;

const std::string RealTimeSynchronizer::ClassName = "RealTimeSynchronizer";

const unsigned PARAM_IDX_BIAS = Block::NumberOfParameters - 1;
const unsigned PARAM_IDX_PERIOD = PARAM_IDX_BIAS + 1;

class RealTimeSynchronizer::impl
{
public:
    double period = 0.01;
    double initialTime;
    unsigned long counter = 0;
};

RealTimeSynchronizer::RealTimeSynchronizer()
    : pImpl{new impl()}
{}

unsigned RealTimeSynchronizer::numberOfParameters()
{
    return Block::numberOfParameters() + 1;
}

bool RealTimeSynchronizer::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata paramMD_period(ParameterType::DOUBLE, PARAM_IDX_PERIOD, 1, 1, "Period");

    bool ok = blockInfo->addParameterMetadata(paramMD_period);

    if (!ok) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool RealTimeSynchronizer::configureSizeAndPorts(BlockInformation* blockInfo)
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

    // OUTPUTS
    // =======
    //
    // No outputs
    //

    if (!blockInfo->setNumberOfOutputPorts(0)) {
        wbtError << "Failed to set output port number.";
        return false;
    }

    return true;
}

bool RealTimeSynchronizer::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    if (!parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    if (!m_parameters.getParameter("Period", pImpl->period)) {
        wbtError << "Failed to get parameter 'period' after its parsing.";
        return false;
    }

    if (pImpl->period <= 0) {
        wbtError << "Period must be greater than 0.";
        return false;
    }

    yarp::os::Network::init();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        wbtError << "YARP server wasn't found active!!";
        return false;
    }

    return true;
}

bool RealTimeSynchronizer::terminate(const BlockInformation* blockInfo)
{
    yarp::os::Network::fini();
    return true;
}

bool RealTimeSynchronizer::output(const BlockInformation* blockInfo)
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
