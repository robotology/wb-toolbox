/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "SimulatorSynchronizer.h"
#include "ClockServer.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
#include <yarp/os/WireLink.h>

#include <ostream>

using namespace wbt;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = Block::NumberOfParameters - 1,
    Period,
    GzClkPort,
    RpcPort
};

// BLOCK PIMPL
// ===========

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
        GazeboYarpPlugins::ClockServer clockServer;
    } rpcData;
};

// BLOCK CLASS
// ===========

SimulatorSynchronizer::SimulatorSynchronizer()
    : pImpl{new impl()}
{}

SimulatorSynchronizer::~SimulatorSynchronizer() = default;

unsigned SimulatorSynchronizer::numberOfParameters()
{
    return Block::numberOfParameters() + 3;
}

std::vector<std::string> SimulatorSynchronizer::additionalBlockOptions()
{
    return std::vector<std::string>(1, blockfactory::core::BlockOptionPrioritizeOrder);
}

bool SimulatorSynchronizer::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> metadata{
        {ParameterType::DOUBLE, ParamIndex::Period, 1, 1, "Period"},
        {ParameterType::STRING, ParamIndex::RpcPort, 1, 1, "RpcPort"},
        {ParameterType::STRING, ParamIndex::GzClkPort, 1, 1, "GazeboClockPort"}};

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            bfError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool SimulatorSynchronizer::configureSizeAndPorts(BlockInformation* blockInfo)
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

bool SimulatorSynchronizer::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!SimulatorSynchronizer::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    std::string serverPortName;
    std::string clientPortName;

    bool ok = true;
    ok = ok && m_parameters.getParameter("Period", pImpl->period);
    ok = ok && m_parameters.getParameter("GazeboClockPort", serverPortName);
    ok = ok && m_parameters.getParameter("RpcPort", clientPortName);

    if (!ok) {
        bfError << "Error reading RPC parameters.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    yarp::os::Network::init();
    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork()) {
        bfError << "Error initializing Yarp network.";
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
            bfError << "Error disconnecting from simulator clock server.";
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
            bfError << "Error connecting to simulator clock server.";
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
