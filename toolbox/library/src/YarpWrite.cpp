/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/YarpWrite.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <BlockFactory/Core/Signal.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/PortReaderBuffer-inl.h>
#include <yarp/sig/Vector.h>

#include <ostream>
#include <tuple>

using namespace wbt::block;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = Block::NumberOfParameters - 1,
    PortName,
    Autoconnect,
    ErrMissingPort,
};

enum InputIndex
{
    Signal = 0,
};

// BLOCK PIMPL
// ===========

class YarpWrite::impl
{
public:
    bool autoconnect = false;
    bool errorOnMissingPort = true;

    std::string destinationPortName;
    yarp::os::BufferedPort<yarp::sig::Vector> port;
    std::unique_ptr<yarp::os::Network> network = nullptr;
};

// BLOCK CLASS
// ===========

YarpWrite::YarpWrite()
    : pImpl{new impl()}
{}

YarpWrite::~YarpWrite() = default;

unsigned YarpWrite::numberOfParameters()
{
    return Block::numberOfParameters() + 3;
}

bool YarpWrite::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> metadata{
        {ParameterType::STRING, ParamIndex::PortName, 1, 1, "PortName"},
        {ParameterType::BOOL, ParamIndex::Autoconnect, 1, 1, "Autoconnect"},
        {ParameterType::BOOL, ParamIndex::ErrMissingPort, 1, 1, "ErrorOnMissingPort"}};

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            bfError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool YarpWrite::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // INPUT
    // =====
    //
    // 1) The signal to stream to the specified yarp port
    //
    // OUTPUT
    // ======
    //
    // No outputs
    //

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
            {InputIndex::Signal, Port::Dimensions{Port::DynamicSize}, Port::DataType::DOUBLE},
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

bool YarpWrite::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    using namespace yarp::os;
    using namespace yarp::sig;

    // PARAMETERS
    // ==========

    if (!YarpWrite::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    std::string portName;

    bool ok = true;
    ok = ok && m_parameters.getParameter("PortName", portName);
    ok = ok && m_parameters.getParameter("Autoconnect", pImpl->autoconnect);
    ok = ok && m_parameters.getParameter("ErrorOnMissingPort", pImpl->errorOnMissingPort);

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

    std::string sourcePortName;

    // Autoconnect: the block opens a temporary output port ..., and it connects to an existing
    //              port portName (which will receive data).
    if (pImpl->autoconnect) {
        sourcePortName = "...";
        pImpl->destinationPortName = portName;
    }
    // Manual connection: the block opens an output port portName, and waits a manual connection to
    // an input port.
    else {
        sourcePortName = portName;
    }

    if (!pImpl->port.open(sourcePortName)) {
        bfError << "Error while opening yarp port.";
        return false;
    }

    if (pImpl->autoconnect) {
        if (!yarp::os::Network::connect(pImpl->port.getName(), pImpl->destinationPortName)) {
            if (pImpl->errorOnMissingPort) {
                bfError << "Failed to connect " << pImpl->port.getName() << " to "
                        << pImpl->destinationPortName << ".";
                return false;
            }
            else {
                bfWarning << "Failed to connect " << pImpl->port.getName() << " to "
                          << pImpl->destinationPortName << ".";
            }
        }
    }

    return true;
}

bool YarpWrite::terminate(const BlockInformation* /*blockInfo*/)
{
    if (pImpl->autoconnect) {
        yarp::os::Network::disconnect(pImpl->port.getName(), pImpl->destinationPortName);
    }

    // Close the port
    pImpl->port.close();

    return true;
}

bool YarpWrite::output(const BlockInformation* blockInfo)
{
    InputSignalPtr signal = blockInfo->getInputPortSignal(InputIndex::Signal);

    if (!signal) {
        bfError << "Input signal not valid.";
        return false;
    }

    yarp::sig::Vector& vectorBuffer = pImpl->port.prepare();
    vectorBuffer.resize(signal->getWidth()); // This should be no-op

    for (unsigned i = 0; i < signal->getWidth(); ++i) {
        vectorBuffer[i] = signal->get<double>(i);
    }

    pImpl->port.write(/*forceStrict=*/true);

    return true;
}
