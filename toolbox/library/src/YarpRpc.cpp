/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
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
    bool shouldReadTimestamp = false;
    bool errorOnMissingPort = true;
    bool firstDataReception = true;
    double timeout = 1.0;
    std::string serverPortName;
    std::string clientPortName;

    std::unique_ptr<yarp::os::Network> network = nullptr;
    yarp::os::RpcClient rpcPort;
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
    return Block::numberOfParameters() + 2;
}

bool YarpRpc::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> metadata{
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
    // PARAMETERS
    // ==========

    if (!YarpRpc::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

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

    using namespace yarp::os;

    // PARAMETERS
    // ==========

    if (!YarpRpc::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    // Temp variable to store rpc command string
    std::string command;

    bool ok = true;
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
    // TODO: Check if the client name can be improved
    pImpl->clientPortName = "/rpcClient" + pImpl->serverPortName;

    if (!pImpl->rpcPort.open(pImpl->clientPortName)) {
        bfError << "Error while opening yarp rpc server port.";
        return false;
    }

    if (!yarp::os::Network::connect(pImpl->clientPortName, pImpl->rpcPort.getName())) {
        if (pImpl->errorOnMissingPort) {
            bfError << "Failed to connect " << pImpl->clientPortName << " to "
                    << pImpl->rpcPort.getName() << ".";
            return false;
        }
        else {
            bfWarning << "Failed to connect " << pImpl->clientPortName  << " to "
                      << pImpl->rpcPort.getName()<< ".";
        }
    }

    // Store rpc command in bottle
    pImpl->rpcCommand.addString(command);

    return true;
}

bool YarpRpc::terminate(const BlockInformation* /*blockInfo*/)
{
    yarp::os::Network::connect(pImpl->clientPortName, pImpl->rpcPort.getName());

    // Close the port
    pImpl->rpcPort.close();

    return true;
}

bool YarpRpc::output(const BlockInformation* blockInfo)
{
    InputSignalPtr triggerSignal = blockInfo->getInputPortSignal(InputIndex::TriggerSignal);

    if (!triggerSignal) {
        bfError << "Input signal not valid.";
        return false;
    }

    yarp::os::Bottle response;

    double trigger = triggerSignal->get<double>(0);

    if (static_cast<bool>(trigger)) {
        pImpl->rpcPort.write(pImpl->rpcCommand, response);
    }

    //TODO: Check if it is useful to forward reponse to an output signal
}
