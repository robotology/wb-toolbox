/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "YarpWrite.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Signal.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

using namespace wbt;

const std::string YarpWrite::ClassName = "YarpWrite";

const unsigned PARAM_IDX_BIAS = Block::NumberOfParameters - 1;
const unsigned PARAM_IDX_PORTNAME = PARAM_IDX_BIAS + 1; // Port name
const unsigned PARAM_IDX_AUTOCONNECT = PARAM_IDX_BIAS + 2; // Autoconnect boolean
const unsigned PARAM_IDX_ERR_NO_PORT = PARAM_IDX_BIAS + 3; // Error on missing port if autoconnect

unsigned YarpWrite::numberOfParameters()
{
    return Block::numberOfParameters() + 3;
}

bool YarpWrite::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata paramMD_portName(ParameterType::STRING, PARAM_IDX_PORTNAME, 1, 1, "PortName");
    ParameterMetadata paramMD_autoconnect(
        ParameterType::BOOL, PARAM_IDX_AUTOCONNECT, 1, 1, "Autoconnect");
    ParameterMetadata paramMD_errMissingPort(
        ParameterType::BOOL, PARAM_IDX_ERR_NO_PORT, 1, 1, "ErrorOnMissingPort");

    bool ok = true;
    ok = ok && blockInfo->addParameterMetadata(paramMD_portName);
    ok = ok && blockInfo->addParameterMetadata(paramMD_autoconnect);
    ok = ok && blockInfo->addParameterMetadata(paramMD_errMissingPort);

    if (!ok) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool YarpWrite::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // INPUT
    // =====
    //
    // 1) The signal to stream to the specified yarp port
    //

    if (!blockInfo->setNumberOfInputPorts(1)) {
        wbtError << "Failed to set input port number to 0.";
        return false;
    }
    blockInfo->setInputPortVectorSize(0, Signal::DynamicSize);
    blockInfo->setInputPortType(0, DataType::DOUBLE);

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

bool YarpWrite::initialize(BlockInformation* blockInfo)
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
    ok = ok && m_parameters.getParameter("Autoconnect", m_autoconnect);
    ok = ok && m_parameters.getParameter("ErrorOnMissingPort", m_errorOnMissingPort);

    if (!ok) {
        wbtError << "Failed to read input parameters.";
        return false;
    }

    std::string sourcePortName;

    // Autoconnect: the block opens a temporary output port ..., and it connects to an existing
    //              port portName (which will receive data).
    if (m_autoconnect) {
        sourcePortName = "...";
        m_destinationPortName = portName;
    }
    // Manual connection: the block opens an output port portName, and waits a manual connection to
    // an input port.
    else {
        sourcePortName = portName;
    }

    m_port = std::unique_ptr<BufferedPort<Vector>>(new BufferedPort<Vector>());

    if (!m_port || !m_port->open(sourcePortName)) {
        wbtError << "Error while opening yarp port.";
        return false;
    }

    if (m_autoconnect) {
        if (!Network::connect(m_port->getName(), m_destinationPortName)) {
            wbtWarning << "Failed to connect " << m_port->getName() << " to "
                       << m_destinationPortName << ".";
            if (m_errorOnMissingPort) {
                wbtError << "Failed connecting ports.";
                return false;
            }
        }
    }

    // Update the size of the signals that were sized dynamically before.
    // At this stage Simulink knows the size.
    const unsigned inputPortWidth = blockInfo->getInputPortWidth(0);
    blockInfo->setInputPortVectorSize(0, inputPortWidth);

    // Initialize the size of the internal buffer handled by m_port
    m_outputVector = m_port->prepare();
    m_outputVector.resize(inputPortWidth);
    return true;
}

bool YarpWrite::terminate(const BlockInformation* /*S*/)
{
    if (m_port) {
        if (m_autoconnect) {
            yarp::os::Network::disconnect(m_port->getName(), m_destinationPortName);
        }
        m_port->close();
    }
    yarp::os::Network::fini();
    return true;
}

bool YarpWrite::output(const BlockInformation* blockInfo)
{
    if (!m_port) {
        return false;
    }

    const Signal signal = blockInfo->getInputPortSignal(0);

    if (!signal.isValid()) {
        wbtError << "Input signal not valid.";
        return false;
    }

    m_outputVector = m_port->prepare();
    m_outputVector.resize(signal.getWidth()); // this should be a no-op

    for (unsigned i = 0; i < signal.getWidth(); ++i) {
        m_outputVector[i] = signal.get<double>(i);
    }

    m_port->write();

    return true;
}
