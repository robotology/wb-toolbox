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
#include "Parameters.h"
#include "Signal.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <iostream>

using namespace wbt;

const std::string YarpRead::ClassName = "YarpRead";

// Parameters
const unsigned PARAM_IDX_BIAS = Block::NumberOfParameters - 1;
const unsigned PARAM_IDX_PORTNAME = PARAM_IDX_BIAS + 1; // port name
const unsigned PARAM_IDX_PORTSIZE = PARAM_IDX_BIAS + 2; // Size of the port you're reading
const unsigned PARAM_IDX_WAITDATA = PARAM_IDX_BIAS + 3; // boolean for blocking reading
const unsigned PARAM_IDX_READ_TS = PARAM_IDX_BIAS + 4; // boolean to stream timestamp
const unsigned PARAM_IDX_AUTOCONNECT = PARAM_IDX_BIAS + 5; // Autoconnect boolean
const unsigned PARAM_IDX_ERR_NO_PORT = PARAM_IDX_BIAS + 6; // Error on missing port if autoconnect
const unsigned PARAM_IDX_TIMEOUT = PARAM_IDX_BIAS + 7; // Timeout if blocking read

unsigned YarpRead::numberOfParameters()
{
    return Block::numberOfParameters() + 7;
}

bool YarpRead::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata paramMD_portName(ParameterType::STRING, PARAM_IDX_PORTNAME, 1, 1, "PortName");
    ParameterMetadata paramMD_signalSize(
        ParameterType::INT, PARAM_IDX_PORTSIZE, 1, 1, "SignalSize");
    ParameterMetadata paramMD_waitData(ParameterType::BOOL, PARAM_IDX_WAITDATA, 1, 1, "WaitData");
    ParameterMetadata paramMD_readTimestamp(
        ParameterType::BOOL, PARAM_IDX_READ_TS, 1, 1, "ReadTimestamp");
    ParameterMetadata paramMD_autoconnect(
        ParameterType::BOOL, PARAM_IDX_AUTOCONNECT, 1, 1, "Autoconnect");
    ParameterMetadata paramMD_timeout(ParameterType::DOUBLE, PARAM_IDX_TIMEOUT, 1, 1, "Timeout");
    ParameterMetadata paramMD_errMissingPort(
        ParameterType::BOOL, PARAM_IDX_ERR_NO_PORT, 1, 1, "ErrorOnMissingPort");

    bool ok = true;
    ok = ok && blockInfo->addParameterMetadata(paramMD_portName);
    ok = ok && blockInfo->addParameterMetadata(paramMD_signalSize);
    ok = ok && blockInfo->addParameterMetadata(paramMD_waitData);
    ok = ok && blockInfo->addParameterMetadata(paramMD_readTimestamp);
    ok = ok && blockInfo->addParameterMetadata(paramMD_autoconnect);
    ok = ok && blockInfo->addParameterMetadata(paramMD_timeout);
    ok = ok && blockInfo->addParameterMetadata(paramMD_errMissingPort);

    if (!ok) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool YarpRead::configureSizeAndPorts(BlockInformation* blockInfo)
{
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
    // 1) Vector with the data read from the port (1 x signalSize)
    // 2) Optional: Timestamp read from the port
    // 3) Optional: If autoconnect is disabled, this output becomes 1 when data starts arriving
    //              (and hence it means that the user connected manually the port)
    //

    if (!parseParameters(blockInfo)) {
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

    if (signalSize < 0) {
        wbtError << "Signal size must be non negative.";
        return false;
    }

    int numberOfOutputPorts = 1;
    numberOfOutputPorts += static_cast<unsigned>(readTimestamp); // timestamp is the second port
    numberOfOutputPorts +=
        static_cast<unsigned>(!autoconnect); // !autoconnect => additional port with 1/0
                                             // depending on the connection status

    if (!blockInfo->setNumberOfOutputPorts(numberOfOutputPorts)) {
        wbtError << "Failed to set output port number.";
        return false;
    }

    blockInfo->setOutputPortVectorSize(0, static_cast<int>(signalSize));
    blockInfo->setOutputPortType(0, DataType::DOUBLE);

    int portIndex = 1;
    if (readTimestamp) {
        blockInfo->setOutputPortVectorSize(portIndex, 2);
        blockInfo->setOutputPortType(portIndex, DataType::DOUBLE);
        portIndex++;
    }
    if (!autoconnect) {
        blockInfo->setOutputPortVectorSize(portIndex, 1);
        blockInfo->setOutputPortType(
            portIndex, DataType::DOUBLE); // use double anyway. Simplifies simulink stuff
        portIndex++;
    }
    return true;
}

bool YarpRead::initialize(BlockInformation* blockInfo)
{
    using namespace yarp::os;
    using namespace yarp::sig;

    if (!parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    Network::init();

    if (!Network::initialized() || !Network::checkNetwork(5.0)) {
        wbtError << "YARP server wasn't found active.";
        return false;
    }

    std::string portName;

    bool ok = true;
    ok = ok && m_parameters.getParameter("PortName", portName);
    ok = ok && m_parameters.getParameter("ReadTimestamp", m_shouldReadTimestamp);
    ok = ok && m_parameters.getParameter("Autoconnect", m_autoconnect);
    ok = ok && m_parameters.getParameter("WaitData", m_blocking);
    ok = ok && m_parameters.getParameter("ErrorOnMissingPort", m_errorOnMissingPort);
    ok = ok && m_parameters.getParameter("Timeout", m_timeout);

    if (!ok) {
        wbtError << "Failed to read input parameters.";
        return false;
    }

    std::string sourcePortName;
    std::string destinationPortName;

    // Autoconnect: the block opens a temporary input port ..., and it connects to an existing
    //              port portName (which is streaming data).
    if (m_autoconnect) {
        sourcePortName = portName;
        destinationPortName = "...";
    }
    // Manual connection: the block opens an input port portName, and waits a manual connection to
    // an output port.
    else {
        destinationPortName = portName;
    }

    m_port = std::unique_ptr<BufferedPort<Vector>>(new BufferedPort<Vector>());

    if (!m_port || !m_port->open(destinationPortName)) {
        wbtError << "Error while opening yarp port.";
        return false;
    }

    if (m_autoconnect) {
        if (!Network::connect(sourcePortName, m_port->getName())) {
            wbtWarning << "Failed to connect " + sourcePortName + " to " + m_port->getName() + ".";
            if (m_errorOnMissingPort) {
                wbtError << "Failed connecting ports.";
                return false;
            }
        }
    }
    return true;
}

bool YarpRead::terminate(const BlockInformation* /*blockInfo*/)
{
    if (m_port) {
        m_port->close();
    }
    yarp::os::Network::fini();
    return true;
}

bool YarpRead::output(const BlockInformation* blockInfo)
{
    int timeStampPortIndex = 0;
    int connectionStatusPortIndex = 0;

    if (m_shouldReadTimestamp) {
        timeStampPortIndex = 1;
    }
    if (!m_autoconnect) {
        connectionStatusPortIndex = timeStampPortIndex + 1;
    }

    yarp::sig::Vector* v = nullptr;

    if (m_blocking) {
        // Initialize the time counter for the timeout
        const double t0 = yarp::os::SystemClock::nowSystem();

        // Loop until something has been read or timeout is reached
        while (true) {
            const int new_bufferSize = m_port->getPendingReads();

            if (new_bufferSize > m_bufferSize) {
                const bool shouldWait = false;
                v = m_port->read(shouldWait);
                m_bufferSize = m_port->getPendingReads();
                break;
            }

            yarp::os::Time::delay(0.0005);
            const double now = yarp::os::Time::now();
            if ((now - t0) > m_timeout) {
                wbtError << "The port didn't receive any data for longer "
                         << "than the configured timeout.";
                return false;
            }
        }
    }
    else {
        bool shouldWait = false;
        v = m_port->read(shouldWait);
    }

    if (v) {
        if (m_shouldReadTimestamp) {
            connectionStatusPortIndex++;
            yarp::os::Stamp timestamp;
            if (!m_port->getEnvelope(timestamp)) {
                wbtError << "Failed to read port envelope (timestamp). Be sure"
                         << " that the input port actually writes this data.";
                return false;
            }

            Signal pY1 = blockInfo->getOutputPortSignal(timeStampPortIndex);
            pY1.set(0, timestamp.getCount());
            pY1.set(1, timestamp.getTime());
        }

        Signal signal = blockInfo->getOutputPortSignal(0);

        if (!signal.isValid()) {
            wbtError << "Output signal not valid.";
            return false;
        }

        // Crop the buffer if it exceeds the OutputPortWidth.
        bool ok =
            signal.setBuffer(v->data(), std::min(signal.getWidth(), static_cast<int>(v->size())));

        if (!ok) {
            wbtError << "Failed to set the output buffer.";
            return false;
        }

        if (!m_autoconnect) {
            Signal statusPort = blockInfo->getOutputPortSignal(connectionStatusPortIndex);
            if (!statusPort.set(0, 1.0)) {
                wbtError << "Failed to write data to output buffer.";
                return false;
            }
        }
    }

    return true;
}
