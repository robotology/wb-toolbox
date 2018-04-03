#include "YarpRead.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Signal.h"

#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <iostream>

using namespace wbt;

const std::string YarpRead::ClassName = "YarpRead";


const double DEFAULT_TIMEOUT = 1.0;

YarpRead::YarpRead()
    : m_autoconnect(false)
    , m_blocking(false)
    , m_shouldReadTimestamp(false)
    , m_errorOnMissingPort(true)
    , m_bufferSize(0)
    , m_timeout(DEFAULT_TIMEOUT)
{}
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

    bool shouldReadTimestamp;
    bool autoconnect;
    double signalSize;

    bool ok = true;

    ok = ok && blockInfo->getBooleanParameterAtIndex(PARAM_IDX_READ_TS, shouldReadTimestamp);
    ok = ok && blockInfo->getBooleanParameterAtIndex(PARAM_IDX_AUTOCONNECT, autoconnect);
    ok = ok && blockInfo->getScalarParameterAtIndex(PARAM_IDX_PORTSIZE, signalSize);

    if (!ok) {
        wbtError << "Failed to read input parameters.";
        return false;
    }

    if (signalSize < 0) {
        wbtError << "Signal size must be non negative.";
        return false;
    }

    int numberOfOutputPorts = 1;
    numberOfOutputPorts +=
        static_cast<unsigned>(shouldReadTimestamp); // timestamp is the second port
    numberOfOutputPorts +=
        static_cast<unsigned>(!autoconnect); // !autoconnect => additional port with 1/0 depending
                                             // on the connection status

    if (!blockInfo->setNumberOfOutputPorts(numberOfOutputPorts)) {
        wbtError << "Failed to set output port number.";
        return false;
    }

    blockInfo->setOutputPortVectorSize(0, static_cast<int>(signalSize));
    blockInfo->setOutputPortType(0, PortDataTypeDouble);

    int portIndex = 1;
    if (shouldReadTimestamp) {
        blockInfo->setOutputPortVectorSize(portIndex, 2);
        blockInfo->setOutputPortType(portIndex, PortDataTypeDouble);
        portIndex++;
    }
    if (!autoconnect) {
        blockInfo->setOutputPortVectorSize(portIndex, 1);
        blockInfo->setOutputPortType(
            portIndex, PortDataTypeInt8); // use double anyway. Simplifies simulink stuff
        portIndex++;
    }
    return true;
}

bool YarpRead::initialize(const BlockInformation* blockInfo)
{
    using namespace yarp::os;
    using namespace yarp::sig;

    Network::init();

    if (!Network::initialized() || !Network::checkNetwork(5.0)) {
        wbtError << "YARP server wasn't found active.";
        return false;
    }

    bool ok = true;

    ok = ok && blockInfo->getBooleanParameterAtIndex(PARAM_IDX_READ_TS, m_shouldReadTimestamp);
    ok = ok && blockInfo->getBooleanParameterAtIndex(PARAM_IDX_AUTOCONNECT, m_autoconnect);
    ok = ok && blockInfo->getBooleanParameterAtIndex(PARAM_IDX_WAITDATA, m_blocking);
    ok = ok && blockInfo->getBooleanParameterAtIndex(PARAM_IDX_ERR_NO_PORT, m_errorOnMissingPort);
    ok = ok && blockInfo->getScalarParameterAtIndex(PARAM_IDX_TIMEOUT, m_timeout);

    if (!ok) {
        Log::getSingleton().error("Failed to read input parameters.");
        return false;
    }

    std::string portName;
    if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_PORTNAME, portName)) {
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

        // Crop the buffer if it exceeds the OutputPortWidth.
        signal.setBuffer(
            v->data(),
            std::min(blockInfo->getOutputPortWidth(0), static_cast<unsigned>(v->size())));

        if (!m_autoconnect) {
            Signal statusPort = blockInfo->getOutputPortSignal(connectionStatusPortIndex);
            statusPort.set(0, 1); // somebody wrote in the port => the port is connected

            // TODO implement a sort of "timeout" paramter
            // At the current state this is equal to timeout = inf
            // Otherwise we can check the timeout and if nobody sent data in the last X secs
            // we set the port to zero again
        }
    }

    return true;
}
