#include "YarpRead.h"

#include "Error.h"
#include "BlockInformation.h"
#include "Signal.h"

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <algorithm>

#define PARAM_IDX_1 1                           // port name
#define PARAM_IDX_2 2                           // Size of the port you're reading
#define PARAM_IDX_3 3                           // boolean for blocking reading
#define PARAM_IDX_4 4                           // boolean to stream timestamp
#define PARAM_IDX_5 5                           // Autoconnect boolean
#define PARAM_IDX_6 6                           // Error on missing port if autoconnect is on boolean

namespace wbt {
    
    std::string YarpRead::ClassName = "YarpRead";

    YarpRead::YarpRead()
    : m_autoconnect(false)
    , m_blocking(false)
    , m_shouldReadTimestamp(false)
    , m_errorOnMissingPort(true)
    , m_port(0) {}

    unsigned YarpRead::numberOfParameters() { return 6; }

    bool YarpRead::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        // Specify I/O
        // INPUTS
        if (!blockInfo->setNumberOfInputPorts(0)) {
            if (error) error->message = "Failed to set input port number to 0";
            return false;
        }
        
        // OUTPUTS
        bool shouldReadTimestamp = blockInfo->getScalarParameterAtIndex(PARAM_IDX_4).booleanData();
        int signalSize = blockInfo->getScalarParameterAtIndex(PARAM_IDX_2).int32Data();
        bool autoconnect = blockInfo->getScalarParameterAtIndex(PARAM_IDX_5).booleanData();

        if (signalSize < 0) {
            if (error) error->message = "Signal size must be non negative";
            return false;
        }

        int numberOfOutputPorts = 1;
        if (shouldReadTimestamp) numberOfOutputPorts++; //timestamp is the second port
        if (!autoconnect) numberOfOutputPorts++; //!autoconnect => additional port with 1/0 depending on the connection status

        if (!blockInfo->setNumberOfOuputPorts(numberOfOutputPorts)) {
            if (error) error->message = "Failed to set output port number";
            return false;
        }

        blockInfo->setOutputPortVectorSize(0, signalSize);
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

    bool YarpRead::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace yarp::os;
        using namespace yarp::sig;

        Network::init();

        if (!Network::initialized() || !Network::checkNetwork(5.0)){
            if (error) error->message = "YARP server wasn't found active!! \n";
            return false;
        }

        m_shouldReadTimestamp = blockInfo->getScalarParameterAtIndex(PARAM_IDX_4).booleanData();
        m_autoconnect = blockInfo->getScalarParameterAtIndex(PARAM_IDX_5).booleanData();
        m_blocking = blockInfo->getScalarParameterAtIndex(PARAM_IDX_3).booleanData();
        m_errorOnMissingPort = blockInfo->getScalarParameterAtIndex(PARAM_IDX_6).booleanData();

        std::string portName;
        if (!blockInfo->getStringParameterAtIndex(PARAM_IDX_1, portName)) {
            if (error) error->message = "Cannot retrieve string from port name parameter";
            return false;
        }

        std::string sourcePortName;
        std::string destinationPortName;

        if (m_autoconnect) {
            sourcePortName = portName;
            destinationPortName = "...";
        } else {
            destinationPortName = portName;
        }

        m_port = new BufferedPort<Vector>();

        if (!m_port || !m_port->open(destinationPortName)) {
            if (error) error->message = "Error while opening yarp port";
            return false;
        }

        if (m_autoconnect) {
            if (!Network::connect(sourcePortName, m_port->getName())) {
                std::cerr << "Failed to connect " << sourcePortName << " to " << m_port->getName() << "\n";
                if (m_errorOnMissingPort) {
                    if (error) error->message = "ERROR connecting ports";
                    return false;
                }
            }
        }
        return true;
    }
    bool YarpRead::terminate(BlockInformation */*S*/, wbt::Error */*error*/)
    {
        if (m_port) {
            m_port->close();
            delete m_port;
            m_port = 0;
        }
        yarp::os::Network::fini();
        return true;
    }
    
    bool YarpRead::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        int timeStampPortIndex = 1;
        int connectionStatusPortIndex = 1;

        yarp::sig::Vector *v = m_port->read(m_blocking); // Read from the port.  Waits until data arrives.
        if (v)
        {
            if (m_shouldReadTimestamp) {
                connectionStatusPortIndex++;
                yarp::os::Stamp timestamp;
                m_port->getEnvelope(timestamp);

                Signal pY1 = blockInfo->getOutputPortSignal(timeStampPortIndex);
                pY1.set(0, timestamp.getCount());
                pY1.set(1, timestamp.getTime());

            }
            Signal signal = blockInfo->getOutputPortSignal(0);
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
}
