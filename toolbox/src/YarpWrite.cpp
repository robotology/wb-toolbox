#include "YarpWrite.h"

#include "Log.h"
#include "BlockInformation.h"
#include "Signal.h"

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

using namespace wbt;

const std::string YarpWrite::ClassName = "YarpWrite";

const unsigned YarpWrite::PARAM_IDX_PORTNAME    = 1; // Port name
const unsigned YarpWrite::PARAM_IDX_AUTOCONNECT = 2; // Autoconnect boolean
const unsigned YarpWrite::PARAM_IDX_ERR_NO_PORT = 3; // Error on missing port if autoconnect is true

YarpWrite::YarpWrite()
: m_autoconnect(false)
, m_errorOnMissingPort(true)
, m_destinationPortName("")
{}

unsigned YarpWrite::numberOfParameters() { return 3; }

bool YarpWrite::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // INPUT
    // =====
    //
    // 1) The signal to stream to the specified yarp port
    //

    if (!blockInfo->setNumberOfInputPorts(1)) {
        Log::getSingleton().error("Failed to set input port number to 0.");
        return false;
    }
    blockInfo->setInputPortVectorSize(0, -1);
    blockInfo->setInputPortType(0, PortDataTypeDouble);

    // OUTPUT
    // ======
    //
    // No outputs
    //

    if (!blockInfo->setNumberOfOutputPorts(0)) {
        Log::getSingleton().error("Failed to set output port number.");
        return false;
    }

    return true;
}

bool YarpWrite::initialize(const BlockInformation* blockInfo)
{
    using namespace yarp::os;
    using namespace yarp::sig;

    Network::init();

    if (!Network::initialized() || !Network::checkNetwork(5.0)){
        Log::getSingleton().error("YARP server wasn't found active!!");
        return false;
    }

    bool ok = true;

    ok = ok && blockInfo->getBooleanParameterAtIndex(PARAM_IDX_AUTOCONNECT, m_autoconnect);
    ok = ok && blockInfo->getBooleanParameterAtIndex(PARAM_IDX_ERR_NO_PORT, m_errorOnMissingPort);

    if (!ok) {
        Log::getSingleton().error("Failed to read input parameters.");
        return false;
    }

    std::string portParameter;
    if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_PORTNAME, portParameter)) {
        Log::getSingleton().error("Error reading port name parameter.");
        return false;
    }

    std::string sourcePortName;

    // Autoconnect: the block opens a temporary output port ..., and it connects to an existing
    //              port portName (which will receive data).
    if (m_autoconnect) {
        sourcePortName = "...";
        m_destinationPortName = portParameter;
    }
    // Manual connection: the block opens an output port portName, and waits a manual connection to an
    //                    input port.
    else {
        sourcePortName = portParameter;
    }

    m_port = std::unique_ptr<BufferedPort<Vector>>(new BufferedPort<Vector>());

    if (!m_port || !m_port->open(sourcePortName)) {
        Log::getSingleton().error("Error while opening yarp port.");
        return false;
    }

    if (m_autoconnect) {
        if (!Network::connect(m_port->getName(), m_destinationPortName)) {
            Log::getSingleton().warning("Failed to connect " +
                                        m_port->getName() +
                                        " to " +
                                        m_destinationPortName);
            if (m_errorOnMissingPort) {
                Log::getSingleton().error("Failed connecting ports.");
                return false;
            }
        }
    }

    // Initialize the size of the internal buffer handled by m_port
    yarp::sig::Vector& outputVector = m_port->prepare();
    outputVector.resize(blockInfo->getInputPortWidth(0));
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
    if (!m_port) return false;
    yarp::sig::Vector& outputVector = m_port->prepare();
    outputVector.resize(blockInfo->getInputPortWidth(0)); //this should be a no-op

    Signal signal = blockInfo->getInputPortSignal(0);

    for (unsigned i = 0; i < blockInfo->getInputPortWidth(0); ++i) {
        outputVector[i] = signal.get<double>(i);
    }

    m_port->write();

    return true;
}
