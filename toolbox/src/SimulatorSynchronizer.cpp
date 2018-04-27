/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "SimulatorSynchronizer.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Parameter.h"
#include "thrift/ClockServer.h"

#include <cmath>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>

using namespace wbt;

const std::string SimulatorSynchronizer::ClassName = "SimulatorSynchronizer";

const unsigned PARAM_IDX_BIAS = Block::NumberOfParameters - 1;
const unsigned PARAM_IDX_PERIOD = PARAM_IDX_BIAS + 1;
const unsigned PARAM_IDX_GZCLK_PORT = PARAM_IDX_BIAS + 2;
const unsigned PARAM_IDX_RPC_PORT = PARAM_IDX_BIAS + 3;

class SimulatorSynchronizer::impl
{
public:
    double period = 0.01;
    bool firstRun = true;

    struct RPCData
    {
        struct
        {
            std::string clientPortName;
            std::string serverPortName;

            unsigned numberOfSteps;
        } configuration;

        yarp::os::Port clientPort;
        gazebo::ClockServer clockServer;
    } rpcData;
};

SimulatorSynchronizer::SimulatorSynchronizer()
    : pImpl{new impl()}
{}

unsigned SimulatorSynchronizer::numberOfParameters()
{
    return Block::numberOfParameters() + 3;
}

std::vector<std::string> SimulatorSynchronizer::additionalBlockOptions()
{
    return std::vector<std::string>(1, wbt::BlockOptionPrioritizeOrder);
}

bool SimulatorSynchronizer::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata paramMD_period(ParameterType::DOUBLE, PARAM_IDX_PERIOD, 1, 1, "Period");
    ParameterMetadata paramMD_rpcPort(ParameterType::STRING, PARAM_IDX_RPC_PORT, 1, 1, "RpcPort");

    ParameterMetadata paramMD_gzclkPort(
        ParameterType::STRING, PARAM_IDX_GZCLK_PORT, 1, 1, "GazeboClockPort");

    bool ok = true;
    ok = ok && blockInfo->addParameterMetadata(paramMD_period);
    ok = ok && blockInfo->addParameterMetadata(paramMD_gzclkPort);
    ok = ok && blockInfo->addParameterMetadata(paramMD_rpcPort);

    if (!ok) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool SimulatorSynchronizer::configureSizeAndPorts(BlockInformation* blockInfo)
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
    // No outputs
    //

    if (!blockInfo->setNumberOfOutputPorts(0)) {
        wbtError << "Failed to set output port number.";
        return false;
    }

    return true;
}

bool SimulatorSynchronizer::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    if (!parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    std::string serverPortName;
    std::string clientPortName;

    bool ok = true;
    ok = ok && m_parameters.getParameter("Period", pImpl->period);
    ok = ok && m_parameters.getParameter("GazeboClockPort", serverPortName);
    ok = ok && m_parameters.getParameter("RpcPort", clientPortName);

    if (!ok) {
        wbtError << "Error reading RPC parameters.";
        return false;
    }

    yarp::os::Network::init();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork()) {
        wbtError << "Error initializing Yarp network.";
        return false;
    }

    pImpl->rpcData.configuration.clientPortName = clientPortName;
    pImpl->rpcData.configuration.serverPortName = serverPortName;

    pImpl->firstRun = true;

    return true;
}

bool SimulatorSynchronizer::terminate(const BlockInformation* /*blockInfo*/)
{
    if (pImpl->rpcData.clientPort.isOpen()) {
        pImpl->rpcData.clockServer.continueSimulation();
        if (!yarp::os::Network::disconnect(pImpl->rpcData.configuration.clientPortName,
                                           pImpl->rpcData.configuration.serverPortName)) {
            wbtError << "Error disconnecting from simulator clock server.";
        }
        pImpl->rpcData.clientPort.close();
    }
    yarp::os::Network::fini();
    return true;
}

bool SimulatorSynchronizer::output(const BlockInformation* /*blockInfo*/)
{
    if (pImpl->firstRun) {
        pImpl->firstRun = false;

        // Connect the ports on first run
        if (!pImpl->rpcData.clientPort.open(pImpl->rpcData.configuration.clientPortName)
            || !yarp::os::Network::connect(pImpl->rpcData.configuration.clientPortName,
                                           pImpl->rpcData.configuration.serverPortName)) {
            wbtError << "Error connecting to simulator clock server.";
            return false;
        }

        // Configure the Wire object from which ClockServer inherits
        pImpl->rpcData.clockServer.yarp().attachAsClient(pImpl->rpcData.clientPort);

        // Calculate how many simulator steps are contained in a single execution of output()
        double stepSize = pImpl->rpcData.clockServer.getStepSize();
        pImpl->rpcData.configuration.numberOfSteps =
            static_cast<unsigned>(pImpl->period / stepSize);

        // Pause the simulation at its very beginning
        pImpl->rpcData.clockServer.pauseSimulation();
    }

    // Simulate the calculated number of steps
    pImpl->rpcData.clockServer.stepSimulationAndWait(pImpl->rpcData.configuration.numberOfSteps);

    return true;
}
