/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "YarpRead.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"
#include "Signal.h"

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
const std::string YarpRead::ClassName = "YarpRead";

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
    bool autoconnect = false;
    bool blocking = false;
    bool shouldReadTimestamp = false;
    bool errorOnMissingPort = true;
    int bufferSize;
    double timeout = 1.0;
    std::string sourcePortName;

    yarp::os::BufferedPort<yarp::sig::Vector> port;
};

// BLOCK CLASS
// ===========

YarpRead::YarpRead()
    : pImpl{new impl()}
{}

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
            wbtError << "Failed to store parameter metadata";
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
        wbtError << "Failed to parse parameters.";
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
        wbtError << "Failed to read input parameters.";
        return false;
    }

    if (signalSize <= 0) {
        wbtError << "Signal size must be non negative.";
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
        wbtError << "Failed to configure input / output ports.";
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
        wbtError << "Failed to parse parameters.";
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
        wbtError << "Failed to read input parameters.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    Network::init();

    if (!Network::initialized() || !Network::checkNetwork(5.0)) {
        wbtError << "YARP server wasn't found active.";
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
        wbtError << "Error while opening yarp port.";
        return false;
    }

    if (pImpl->autoconnect) {
        if (!Network::connect(pImpl->sourcePortName, pImpl->port.getName())) {
            if (pImpl->errorOnMissingPort) {
                wbtError << "Failed to connect " + pImpl->sourcePortName + " to "
                                + pImpl->port.getName() + ".";
                return false;
            }
            else {
                wbtWarning << "Failed to connect " + pImpl->sourcePortName + " to "
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
        while (true) {
            const int new_bufferSize = pImpl->port.getPendingReads();

            if (new_bufferSize > pImpl->bufferSize) {
                const bool shouldWait = false;
                vectorBuffer = pImpl->port.read(shouldWait);
                pImpl->bufferSize = pImpl->port.getPendingReads();
                break;
            }

            yarp::os::Time::delay(0.0005);
            const double now = yarp::os::Time::now();
            if ((now - t0) > pImpl->timeout) {
                wbtError << "The port didn't receive any data for longer "
                         << "than the configured timeout.";
                return false;
            }
        }
    }
    else {
        bool shouldWait = false;
        vectorBuffer = pImpl->port.read(shouldWait);
    }

    if (vectorBuffer) {
        if (pImpl->shouldReadTimestamp) {
            yarp::os::Stamp timestamp;
            if (!pImpl->port.getEnvelope(timestamp)) {
                wbtError << "Failed to read port envelope (timestamp). Be sure"
                         << " that the input port actually writes this data.";
                return false;
            }

            OutputSignalPtr timestampSignal = blockInfo->getOutputPortSignal(OutputIndex_Timestamp);
            if (!timestampSignal) {
                wbtError << "Output signal not valid.";
                return false;
            }

            timestampSignal->set(0, timestamp.getCount());
            timestampSignal->set(1, timestamp.getTime());
        }

        OutputSignalPtr signal = blockInfo->getOutputPortSignal(OutputIndex::Signal);

        if (!signal) {
            wbtError << "Output signal not valid.";
            return false;
        }

        // Crop the buffer if it exceeds the OutputPortWidth.
        if (!signal->setBuffer(
                vectorBuffer->data(),
                std::min(signal->getWidth(), static_cast<int>(vectorBuffer->size())))) {
            wbtError << "Failed to set the output buffer.";
            return false;
        }

        if (!pImpl->autoconnect) {
            OutputSignalPtr statusPort = blockInfo->getOutputPortSignal(OutputIndex_IsConnected);
            if (!statusPort) {
                wbtError << "Output signal not valid.";
                return false;
            }

            if (!statusPort->set(0, 1)) {
                wbtError << "Failed to write data to output buffer.";
                return false;
            }
        }
    }

    return true;
}
