#include "YarpWrite.h"

#include "Error.h"
#include "BlockInformation.h"
#include "Signal.h"

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#define PARAM_IDX_1 1                           // port name
#define PARAM_IDX_2 2                           // Autoconnect boolean
#define PARAM_IDX_3 3                           // Error on missing port if autoconnect is on boolean

namespace wbt {
    
    std::string YarpWrite::ClassName = "YarpWrite";

    YarpWrite::YarpWrite()
    : m_autoconnect(false)
    , m_errorOnMissingPort(true)
    , m_destinationPortName("")
    , m_port(0) {}
    
    unsigned YarpWrite::numberOfParameters() { return 3; }

    bool YarpWrite::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (!blockInfo->setNumberOfInputPorts(1)) {
            if (error) error->message = "Failed to set input port number to 0";
            return false;
        }
        blockInfo->setInputPortVectorSize(0, -1);
        blockInfo->setInputPortType(0, PortDataTypeDouble);

        if (!blockInfo->setNumberOfOuputPorts(0)) {
            if (error) error->message = "Failed to set output port number";
            return false;
        }
        return true;
    }

    bool YarpWrite::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace yarp::os;
        using namespace yarp::sig;

        Network::init();

        if (!Network::initialized() || !Network::checkNetwork(5.0)){
            if (error) error->message = "YARP server wasn't found active!! \n";
            return false;
        }

        m_autoconnect = blockInfo->getScalarParameterAtIndex(PARAM_IDX_2).booleanData();
        m_errorOnMissingPort = blockInfo->getScalarParameterAtIndex(PARAM_IDX_3).booleanData();

        std::string portParameter;
        if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_1, portParameter)) {
            if (error) error->message = "Error reading port name parameter";
            return false;
        }

        std::string sourcePortName;

        if (m_autoconnect) {
            sourcePortName = "...";
            m_destinationPortName = portParameter;
        } else {
            sourcePortName = portParameter;
        }

        m_port = new BufferedPort<Vector>();

        if (!m_port || !m_port->open(sourcePortName)) {
            if (error) error->message = "Error while opening yarp port";
            return false;
        }

        if (m_autoconnect) {
            if (!Network::connect(m_port->getName(), m_destinationPortName)) {
                if (m_errorOnMissingPort) {
                    if (error) error->message ="Failed to connect " + m_port->getName() + " to " + m_destinationPortName;
                    return false;
                }
            }
        }

        //prepare the first object allocation
        yarp::sig::Vector &outputVector = m_port->prepare();
        outputVector.resize(blockInfo->getInputPortWidth(0));
        return true;
    }

    bool YarpWrite::terminate(BlockInformation */*S*/, wbt::Error */*error*/)
    {
        if (m_port) {
            if (m_autoconnect)
                yarp::os::Network::disconnect(m_port->getName(), m_destinationPortName);
            m_port->close();
            delete m_port;
            m_port = 0;
        }
        yarp::os::Network::fini();
        return true;
    }
    
    bool YarpWrite::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        if (!m_port) return false;
        yarp::sig::Vector &outputVector = m_port->prepare();
        outputVector.resize(blockInfo->getInputPortWidth(0)); //this should be a no-op

        Signal signal = blockInfo->getInputPortSignal(0);
        for (unsigned i = 0; i < blockInfo->getInputPortWidth(0); ++i) {
            outputVector[i] = signal.get(i).doubleData();
        }

        m_port->write();

        return true;
    }
}
