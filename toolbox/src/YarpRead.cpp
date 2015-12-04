#include "YarpRead.h"
#include "Error.h"

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#define PARAM_IDX_1 1                           // port name
#define PARAM_IDX_2 2                           // Size of the port you're reading
#define PARAM_IDX_3 3                           // boolean for blocking reading
#define PARAM_IDX_4 4                           // boolean to stream timestamp
#define PARAM_IDX_5 5                           // Autoconnect boolean
#define PARAM_IDX_6 6                           // Error on missing port if autoconnect is on boolean
#define GET_OPT_SIGNAL_SIZE mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_2))
#define GET_OPT_BLOCKING  mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_3))
#define GET_OPT_TIMESTAMP mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_4))
#define GET_OPT_AUTOCONNECT mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_5))
#define GET_OPT_ERROR_ON_MISSING_PORT mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_6))

namespace wbt {
    
    std::string YarpRead::ClassName = "YarpRead";

    YarpRead::YarpRead() {}
    YarpRead::~YarpRead() {}
    
    unsigned YarpRead::numberOfParameters() { return 6; }

    bool YarpRead::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        // Specify I/O
        // INPUTS
        if(!ssSetNumInputPorts(S, 0)) {
            if (error) error->message = "Failed to set input port number to 0";
            return false;
        }
        
        // OUTPUTS
        int_T shouldReadTimestamp = GET_OPT_TIMESTAMP;
        int_T signalSize = GET_OPT_SIGNAL_SIZE;
        int_T autoconnect = GET_OPT_AUTOCONNECT;

        if (signalSize < 0) {
            if (error) error->message = "Signal size must be non negative";
            return false;
        }

        int numberOfOutputPorts = 1;
        if (shouldReadTimestamp) numberOfOutputPorts++; //timestamp is the second port
        if (!autoconnect) numberOfOutputPorts++; //!autoconnect => additional port with 1/0 depending on the connection status

        if (!ssSetNumOutputPorts(S, numberOfOutputPorts)) {
            if (error) error->message = "Failed to set output port number";
            return false;
        }

        ssSetOutputPortWidth(S, 0, signalSize);
        ssSetOutputPortDataType(S, 0, SS_DOUBLE);

        int portIndex = 1;
        if (shouldReadTimestamp) {
            ssSetOutputPortWidth(S, portIndex, 2);
            ssSetOutputPortDataType(S, portIndex, SS_DOUBLE);
            portIndex++;
        }
        if (!autoconnect) {
            ssSetOutputPortWidth(S, portIndex, 1);
            ssSetOutputPortDataType(S, portIndex, SS_INT8); //use double anyway. Simplifies simulink stuff
            portIndex++;
        }
        return true;
    }

    bool YarpRead::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;
        using namespace yarp::sig;

        Network::init();

        if (!Network::initialized() || !Network::checkNetwork(5.0)){
            if (error) error->message = "YARP server wasn't found active!! \n";
            return false;
        }

        m_shouldReadTimestamp = GET_OPT_TIMESTAMP;
        m_autoconnect = GET_OPT_AUTOCONNECT;
        m_blocking = GET_OPT_BLOCKING;
        m_errorOnMissingPort = GET_OPT_ERROR_ON_MISSING_PORT;

        std::string portName;
        if (!Block::readStringParameterAtIndex(S, PARAM_IDX_1, portName)) {
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
                mexPrintf("Failed to connect %s to %s\n", sourcePortName.c_str(), m_port->getName().c_str());
                if (m_errorOnMissingPort) {
                    if (error) error->message = "ERROR connecting ports";
                    return false;
                }
            }
        }
        return true;
    }
    bool YarpRead::terminate(SimStruct *S, wbt::Error *error) 
    {
        if (m_port) {
            m_port->close();
            delete m_port;
            m_port = 0;
        }
        yarp::os::Network::fini();
        return true;
    }
    
    bool YarpRead::output(SimStruct *S, wbt::Error *error) 
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

                real_T *pY1 = ssGetOutputPortRealSignal(S, timeStampPortIndex);
                pY1[0] = (real_T)(timestamp.getCount());
                pY1[1] = (real_T)(timestamp.getTime());
            }
            real_T *signal = ssGetOutputPortRealSignal(S, 0);
            int_T widthPort = ssGetOutputPortWidth(S, 0);
            for (int i = 0; i < std::min(widthPort, (int_T)v->size()); i++) {
                signal[i] = (*v)[i];
            }

            if (!m_autoconnect) {
                unsigned char *statusPort = (unsigned char*)ssGetOutputPortSignal(S, connectionStatusPortIndex);
                statusPort[0] = 1; //somebody wrote in the port => the port is connected
                //TODO implement a sort of "timeout" paramter
                //At the current state this is equal to timeout = inf
                //Otherwise we can check the timeout and if nobody sent data in the last X secs
                //we set the port to zero again
            }
        }
        return true;
    }

}