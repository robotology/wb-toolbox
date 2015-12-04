#include "YarpWrite.h"
#include "Error.h"

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#define PARAM_IDX_1 1                           // port name
#define PARAM_IDX_2 2                           // Autoconnect boolean
#define PARAM_IDX_3 3                           // Error on missing port if autoconnect is on boolean
#define GET_OPT_AUTOCONNECT mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_2))
#define GET_OPT_ERROR_ON_MISSING_PORT mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_3))

namespace wbt {
    
    std::string YarpWrite::ClassName = "YarpWrite";

    YarpWrite::YarpWrite()
    : m_autoconnect(false)
    , m_errorOnMissingPort(true)
    , m_destinationPortName("") {}

    YarpWrite::~YarpWrite() {}
    
    unsigned YarpWrite::numberOfParameters() { return 3; }

    bool YarpWrite::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        if(!ssSetNumInputPorts(S, 1)) {
            if (error) error->message = "Failed to set input port number to 0";
            return false;
        }
        ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
        ssSetInputPortDataType (S, 0, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough (S, 0, 1);

        if (!ssSetNumOutputPorts(S, 0)) {
            if (error) error->message = "Failed to set output port number";
            return false;
        }
        return true;
    }

    bool YarpWrite::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;
        using namespace yarp::sig;

        Network::init();

        if (!Network::initialized() || !Network::checkNetwork(5.0)){
            if (error) error->message = "YARP server wasn't found active!! \n";
            return false;
        }

        m_autoconnect = GET_OPT_AUTOCONNECT;
        m_errorOnMissingPort = GET_OPT_ERROR_ON_MISSING_PORT;

        std::string portParameter;
        if (!Block::readStringParameterAtIndex(S, PARAM_IDX_1, portParameter)) {
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
        outputVector.resize(ssGetInputPortWidth(S, 0));
        return true;
    }

    bool YarpWrite::terminate(SimStruct *S, wbt::Error *error)
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
    
    bool YarpWrite::output(SimStruct *S, wbt::Error *error)
    {
        if (!m_port) return false;
        yarp::sig::Vector &outputVector = m_port->prepare();
        outputVector.resize(ssGetInputPortWidth(S, 0));

        InputRealPtrsType signal = ssGetInputPortRealSignalPtrs(S, 0);
        for (unsigned i = 0; i < ssGetInputPortWidth(S, 0); ++i) {
            outputVector[i] = *signal[i];
        }

        m_port->write();

        return true;
    }
}
