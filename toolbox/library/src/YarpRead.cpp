/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "YarpRead.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <BlockFactory/Core/Signal.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/PortReaderBuffer-inl.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/SystemClock.h>
#include <yarp/os/Time.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <iostream>
#include <string>
#include <tuple>

using namespace wbt;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = Block::NumberOfParameters - 1,
    PortName,
    PortSize,
    WaitData,
    ReadTimestamp,
    Autoconnect,
    ErrMissingPort,
    Timeout
};

enum OutputIndex
{
    Signal = 0,
    // Other optional outputs
};

static int OutputIndex_Timestamp = OutputIndex::Signal;
static int OutputIndex_IsConnected = OutputIndex::Signal;

// BLOCK PIMPL
// ===========

class YarpRead::impl
{
public:
    int bufferSize;
    bool autoconnect = false;
    bool blocking = false;
    bool shouldReadTimestamp = false;
    bool errorOnMissingPort = true;
    bool firstDataReception = true;
    double timeout = 1.0;
    std::string sourcePortName;

    yarp::os::BufferedPort<yarp::sig::Vector> port;
};

// BLOCK CLASS
// ===========

YarpRead::YarpRead()
    : pImpl{new impl()}
{}

YarpRead::~YarpRead() = default;

unsigned YarpRead::numberOfParameters()
{
    return Block::numberOfParameters() + 7;
}

bool YarpRead::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> metadata{
        {ParameterType::STRING, ParamIndex::PortName, 1, 1, "PortName"},
        {ParameterType::INT, ParamIndex::PortSize, 1, 1, "SignalSize"},
        {ParameterType::BOOL, ParamIndex::WaitData, 1, 1, "WaitData"},
        {ParameterType::BOOL, ParamIndex::ReadTimestamp, 1, 1, "ReadTimestamp"},
        {ParameterType::BOOL, ParamIndex::Autoconnect, 1, 1, "Autoconnect"},
        {ParameterType::DOUBLE, ParamIndex::Timeout, 1, 1, "Timeout"},
        {ParameterType::BOOL, ParamIndex::ErrMissingPort, 1, 1, "ErrorOnMissingPort"}};

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            bfError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool YarpRead::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // PARAMETERS
    // ==========

    if (!YarpRead::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    bool readTimestamp = false;
    bool autoconnect = false;
    int signalSize = 0;

    bool ok = true;
    ok = ok && m_parameters.getParameter("ReadTimestamp", readTimestamp);
    ok = ok && m_parameters.getParameter("Autoconnect", autoconnect);
    ok = ok && m_parameters.getParameter("SignalSize", signalSize);

    if (!ok) {
        bfError << "Failed to read input parameters.";
        return false;
    }

    if (signalSize <= 0) {
        bfError << "Signal size must be non negative.";
        return false;
    }

    // INPUTS
    // ======
    //
    // No inputs
    //
    // OUTPUTS
    // =======
    //
    // 1) Vector with the data read from the port (1 x signalSize)
    // 2) Optional: Timestamp read from the port
    // 3) Optional: If autoconnect is disabled, this output becomes 1 when data starts arriving
    //              (and hence it means that the user connected manually the port)
    //

    // Optional outputs
    if (readTimestamp) {
        OutputIndex_Timestamp = OutputIndex::Signal + 1;
    }
    if (!autoconnect) {
        OutputIndex_IsConnected = OutputIndex_Timestamp + 1;
    }

    BlockInformation::IOData ioData;
    ioData.output.emplace_back(OutputIndex::Signal, std::vector<int>{signalSize}, DataType::DOUBLE);

    if (readTimestamp) {
        ioData.output.emplace_back(OutputIndex_Timestamp, std::vector<int>{2}, DataType::DOUBLE);
    }
    if (!autoconnect) {
        // Use double anyway even if it is a bool signal.
        ioData.output.emplace_back(OutputIndex_IsConnected, std::vector<int>{1}, DataType::DOUBLE);
    }

    if (!blockInfo->setIOPortsData(ioData)) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool YarpRead::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    using namespace yarp::os;
    using namespace yarp::sig;

    // PARAMETERS
    // ==========

    if (!YarpRead::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    std::string portName;

    bool ok = true;
    ok = ok && m_parameters.getParameter("PortName", portName);
    ok = ok && m_parameters.getParameter("ReadTimestamp", pImpl->shouldReadTimestamp);
    ok = ok && m_parameters.getParameter("Autoconnect", pImpl->autoconnect);
    ok = ok && m_parameters.getParameter("WaitData", pImpl->blocking);
    ok = ok && m_parameters.getParameter("ErrorOnMissingPort", pImpl->errorOnMissingPort);
    ok = ok && m_parameters.getParameter("Timeout", pImpl->timeout);

    if (!ok) {
        bfError << "Failed to read input parameters.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    Network::init();

    if (!Network::initialized() || !Network::checkNetwork(5.0)) {
        bfError << "YARP server wasn't found active.";
        return false;
    }

    std::string destinationPortName;

    // Autoconnect: the block opens a temporary input port ..., and it connects to an existing
    //              port portName (which is streaming data).
    if (pImpl->autoconnect) {
        pImpl->sourcePortName = portName;
        destinationPortName = "...";
    }
    // Manual connection: the block opens an input port portName, and waits a manual connection to
    // an output port.
    else {
        destinationPortName = portName;
    }

    if (!pImpl->port.open(destinationPortName)) {
        bfError << "Error while opening yarp port.";
        return false;
    }

    if (pImpl->autoconnect) {
        if (!Network::connect(pImpl->sourcePortName, pImpl->port.getName())) {
            if (pImpl->errorOnMissingPort) {
                bfError << "Failed to connect " + pImpl->sourcePortName + " to "
                               + pImpl->port.getName() + ".";
                return false;
            }
            else {
                bfWarning << "Failed to connect " + pImpl->sourcePortName + " to "
                                 + pImpl->port.getName() + ".";
            }
        }
    }
    return true;
}

bool YarpRead::terminate(const BlockInformation* /*blockInfo*/)
{
    if (pImpl->autoconnect) {
        yarp::os::Network::disconnect(pImpl->port.getName(), pImpl->sourcePortName);
    }

    // Close the port
    pImpl->port.close();

    yarp::os::Network::fini();
    return true;
}

bool YarpRead::output(const BlockInformation* blockInfo)
{
    yarp::sig::Vector* vectorBuffer = nullptr;

    if (pImpl->blocking) {
        // Initialize the time counter for the timeout
        const double t0 = yarp::os::SystemClock::nowSystem();

        // Loop until something has been read or timeout is reached
        while (!vectorBuffer) {
            // This reads always the newest message even if other have been buffered
            vectorBuffer = pImpl->port.read(/*shouldWait=*/false);

            if (!vectorBuffer) {
                yarp::os::Time::delay(0.0005);
                const double now = yarp::os::SystemClock::nowSystem();
                if ((now - t0) > pImpl->timeout) {
                    bfError << "The port didn't receive any data for longer "
                            << "than the configured timeout.";
                    return false;
                }
            }
        }
    }
    else {
        vectorBuffer = pImpl->port.read(/*shouldWait=*/false);
    }

    if (vectorBuffer) {
        if (pImpl->shouldReadTimestamp) {
            yarp::os::Stamp timestamp;
            if (!pImpl->port.getEnvelope(timestamp)) {
                bfError << "Failed to read port envelope (timestamp). Be sure"
                        << " that the input port actually writes this data.";
                return false;
            }

            OutputSignalPtr timestampSignal = blockInfo->getOutputPortSignal(OutputIndex_Timestamp);
            if (!timestampSignal) {
                bfError << "Output signal not valid.";
                return false;
            }

            timestampSignal->set(0, timestamp.getCount());
            timestampSignal->set(1, timestamp.getTime());
        }

        OutputSignalPtr signal = blockInfo->getOutputPortSignal(OutputIndex::Signal);

        if (!signal) {
            bfError << "Output signal not valid.";
            return false;
        }

        if (vectorBuffer->size() != signal->getWidth()) {
            bfError << "Size of received data from " << pImpl->port.getName() << " ("
                    << vectorBuffer->size() << ")"
                    << " does not match with output signal width (" << signal->getWidth() << ").";
            return false;
        }

        if (!signal->setBuffer(vectorBuffer->data(), vectorBuffer->size())) {
            bfError << "Failed to set the output buffer.";
            return false;
        }

        // Raise trigger for notifying data reception
        // TODO: add the initializeInitialConditions method?
        if (!pImpl->autoconnect && pImpl->firstDataReception) {

            pImpl->firstDataReception = false;

            OutputSignalPtr statusPort = blockInfo->getOutputPortSignal(OutputIndex_IsConnected);
            if (!statusPort) {
                bfError << "Output signal not valid.";
                return false;
            }

            if (!statusPort->set(0, 1)) {
                bfError << "Failed to write data to output buffer.";
                return false;
            }
        }
    }

    return true;
}
