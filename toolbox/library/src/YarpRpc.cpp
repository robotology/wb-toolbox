/*
 * Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/YarpRpc.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <BlockFactory/Core/Signal.h>

#include <yarp/os/Network.h>
#include <yarp/os/RpcClient.h>
#include <yarp/os/Bottle.h>

#include <string>

using namespace wbt::block;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = Block::NumberOfParameters - 1,
    RisingEdgeTrigger,
    PortName,
    RpcCommand,
};

enum InputIndex
{
    TriggerSignal = 0,
};

// BLOCK PIMPL
// ===========

class YarpRpc::impl
{
public:
    double timeout = 0.25;
    std::string serverPortName;
    std::string clientPortName;

    std::unique_ptr<yarp::os::Network> network = nullptr;
    bool previousTrigger = false;
    bool risingEdgeTrigger;
    yarp::os::RpcClient rpcClientPort;
    yarp::os::Bottle rpcCommand;
};

// BLOCK CLASS
// ===========

YarpRpc::YarpRpc()
    : pImpl{new impl()}
{}

YarpRpc::~YarpRpc() = default;

unsigned YarpRpc::numberOfParameters()
{
    return Block::numberOfParameters() + 3;
}

bool YarpRpc::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> metadata{
        {ParameterType::BOOL, ParamIndex::RisingEdgeTrigger, 1, 1, "RisingEdgeTrigger"},
        {ParameterType::STRING, ParamIndex::PortName, 1, 1, "PortName"},
        {ParameterType::STRING, ParamIndex::RpcCommand, 1, 1, "RpcCommand"}
    };

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            bfError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool YarpRpc::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // INPUT
    // =====
    //
    // 1) Trigger signal
    //
    // OUTPUT
    // ======
    //
    // No outputs
    //

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
            {InputIndex::TriggerSignal, Port::Dimensions{1}, Port::DataType::DOUBLE},
        },
        {
            // Outputs
        });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool YarpRpc::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!YarpRpc::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    // Temp variable to store rpc command string
    std::string command;

    bool ok = true;
    ok = ok && m_parameters.getParameter("RisingEdgeTrigger", pImpl->risingEdgeTrigger);
    ok = ok && m_parameters.getParameter("PortName", pImpl->serverPortName);
    ok = ok && m_parameters.getParameter("RpcCommand", command);

    if (!ok) {
        bfError << "Failed to read input parameters.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    pImpl->network = std::make_unique<yarp::os::Network>();

    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        bfError << "YARP server wasn't found active.";
        return false;
    }

    // Open a rpc client port
    pImpl->clientPortName = "...";

    if (!pImpl->rpcClientPort.open(pImpl->clientPortName)) {
        bfError << "Error while opening yarp rpc server port.";
        return false;
    }

    if (!yarp::os::Network::connect(pImpl->rpcClientPort.getName(), pImpl->serverPortName)) {
        bfError << "Failed to connect " << pImpl->serverPortName << " to "
                << pImpl->rpcClientPort.getName() << ".";
        return false;
    }

    // Store rpc command in bottle
    pImpl->rpcCommand.fromString(command);

    return true;
}

bool YarpRpc::terminate(const BlockInformation* /*blockInfo*/)
{
    if(pImpl->rpcClientPort.isWriting()) {
        bfWarning << "Interrupting rpc port";
        pImpl->rpcClientPort.interrupt();
    }

    yarp::os::Network::disconnect(pImpl->rpcClientPort.getName(), pImpl->serverPortName);

    // Close the port
    pImpl->rpcClientPort.close();

    return true;
}

bool YarpRpc::output(const BlockInformation* blockInfo)
{
    InputSignalPtr triggerSignal = blockInfo->getInputPortSignal(InputIndex::TriggerSignal);

    if (!triggerSignal) {
        bfError << "Input signal not valid.";
        return false;
    }

    bool rpcStatus = true;

    //TODO: Update to getting bool type after
    //testing boolean support more thoroughly
    double trigger = triggerSignal->get<double>(0);

    if (pImpl->risingEdgeTrigger) {

        if (trigger && !pImpl->previousTrigger) {

            rpcStatus = sendRpcCommand();

        }

    }
    else if (static_cast<bool>(trigger)) {

        rpcStatus = sendRpcCommand();

    }

    //Update previous trigger
    pImpl->previousTrigger = trigger;

    //TODO: Check if it is useful to forward reponse to an output signal
    return rpcStatus;
}

bool YarpRpc::sendRpcCommand()
{
    if (pImpl->rpcClientPort.isWriting()) {
        bfError << "YarpRpc client port is blocked in writing";
        return false;
    }

    yarp::os::Bottle response;

    // Initialize the time counter for the timeout
    const double t0 = yarp::os::SystemClock::nowSystem();

    while(!pImpl->rpcClientPort.write(pImpl->rpcCommand, response)) {

        yarp::os::Time::delay(0.0005);
        const double now = yarp::os::SystemClock::nowSystem();

        if((now - t0) > pImpl->timeout) {
            bfError << "Failed to send the rpc command to the server "
                    << " with in the configured timeout";
            return false;
        }
    }

    bfWarning << response.toString().c_str();

    return true;
}
