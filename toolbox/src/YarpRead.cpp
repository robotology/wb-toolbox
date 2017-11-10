#include "YarpRead.h"

#include "Log.h"
#include "BlockInformation.h"
#include "Signal.h"

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <iostream>

using namespace wbt;

const std::string YarpRead::ClassName = "YarpRead";

const unsigned YarpRead::PARAM_IDX_PORTNAME    = 1; // port name
const unsigned YarpRead::PARAM_IDX_PORTSIZE    = 2; // Size of the port you're reading
const unsigned YarpRead::PARAM_IDX_WAITDATA    = 3; // boolean for blocking reading
const unsigned YarpRead::PARAM_IDX_READ_TS     = 4; // boolean to stream timestamp
const unsigned YarpRead::PARAM_IDX_AUTOCONNECT = 5; // Autoconnect boolean
const unsigned YarpRead::PARAM_IDX_ERR_NO_PORT = 6; // Error on missing port if autoconnect is on boolean

YarpRead::YarpRead()
: m_autoconnect(false)
, m_blocking(false)
, m_shouldReadTimestamp(false)
, m_errorOnMissingPort(true)
, m_port(0)
{}

unsigned YarpRead::numberOfParameters() { return 6; }

bool YarpRead::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // INPUTS
    // ======
    //
    // No inputs
    //

    if (!blockInfo->setNumberOfInputPorts(0)) {
        Log::getSingleton().error("Failed to set input port number to 0.");
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

    bool ok = false;

    ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_READ_TS, shouldReadTimestamp);
    ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_AUTOCONNECT, autoconnect);
    ok = ok & blockInfo->getScalarParameterAtIndex(PARAM_IDX_PORTSIZE, signalSize);

    if (!ok) {
        Log::getSingleton().error("Failed to read input parameters.");
        return false;
    }

    if (signalSize < 0) {
        Log::getSingleton().error("Signal size must be non negative.");
        return false;
    }

    int numberOfOutputPorts = 1;
    numberOfOutputPorts += static_cast<unsigned>(shouldReadTimestamp); //timestamp is the second port
    numberOfOutputPorts += static_cast<unsigned>(!autoconnect); //!autoconnect => additional port with 1/0 depending on the connection status

    if (!blockInfo->setNumberOfOutputPorts(numberOfOutputPorts)) {
        Log::getSingleton().error("Failed to set output port number.");
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
        blockInfo->setOutputPortType(portIndex, PortDataTypeInt8); //use double anyway. Simplifies simulink stuff
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
        Log::getSingleton().error("YARP server wasn't found active!!");
        return false;
    }

    bool ok = true;

    ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_READ_TS, m_shouldReadTimestamp);
    ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_AUTOCONNECT, m_autoconnect);
    ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_WAITDATA, m_blocking);
    ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_ERR_NO_PORT, m_errorOnMissingPort);

    if (!ok) {
        Log::getSingleton().error("Failed to read input parameters.");
        return false;
    }

    std::string portName;
    if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_PORTNAME, portName)) {
        Log::getSingleton().error("Cannot retrieve string from port name parameter.");
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
    // Manual connection: the block opens an input port portName, and waits a manual connection to an
    //                    output port.
    else {
        destinationPortName = portName;
    }

    m_port = new BufferedPort<Vector>();

    if (!m_port || !m_port->open(destinationPortName)) {
        Log::getSingleton().error("Error while opening yarp port.");
        return false;
    }

    if (m_autoconnect) {
        if (!Network::connect(sourcePortName, m_port->getName())) {
            Log::getSingleton().warning("Failed to connect " +
                                        sourcePortName +
                                        " to "
                                        + m_port->getName());
            if (m_errorOnMissingPort) {
                Log::getSingleton().error("Failed connecting ports.");
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
        delete m_port;
        m_port = nullptr;
    }
    yarp::os::Network::fini();
    return true;
}

bool YarpRead::output(const BlockInformation* blockInfo)
{
    int timeStampPortIndex = 1;
    int connectionStatusPortIndex = 1;

    yarp::sig::Vector* v = m_port->read(m_blocking); // Read from the port.  Waits until data arrives.

    if (v) {
        if (m_shouldReadTimestamp) {
            connectionStatusPortIndex++;
            yarp::os::Stamp timestamp;
            m_port->getEnvelope(timestamp);

            Signal pY1 = blockInfo->getOutputPortSignal(timeStampPortIndex);
            pY1.set(0, timestamp.getCount());
            pY1.set(1, timestamp.getTime());
        }

        Signal signal = blockInfo->getOutputPortSignal(0);

        // Crop the buffer if it exceeds the OutputPortWidth.
        signal.setBuffer(v->data(), std::min(blockInfo->getOutputPortWidth(0), (unsigned)v->size()));

        if (!m_autoconnect) {
            Signal statusPort = blockInfo->getOutputPortSignal(connectionStatusPortIndex);
            statusPort.set(0, 1); //somebody wrote in the port => the port is connected

            //TODO implement a sort of "timeout" paramter
            //At the current state this is equal to timeout = inf
            //Otherwise we can check the timeout and if nobody sent data in the last X secs
            //we set the port to zero again
        }
    }

    return true;
}
