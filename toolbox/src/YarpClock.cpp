#include "YarpClock.h"

#include "Error.h"
#include "BlockInformation.h"
#include "Signal.h"

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

namespace wbt {

    std::string YarpClock::ClassName = "YarpClock";

    unsigned YarpClock::numberOfParameters() { return 0; }

    bool YarpClock::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        // Specify I/O
        // INPUTS
        if (!blockInfo->setNumberOfInputPorts(0)) {
            if (error) error->message = "Failed to set input port number to 0";
            return false;
        }

        // OUTPUTS
        if (!blockInfo->setNumberOfOuputPorts(1)) {
            if (error) error->message = "Failed to set output port number";
            return false;
        }

        blockInfo->setOutputPortVectorSize(0, 1);
        blockInfo->setOutputPortType(0, PortDataTypeDouble);

        return true;
    }

    bool YarpClock::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        yarp::os::Network::init();

        if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)){
            if (error) error->message = "YARP server wasn't found active!! \n";
            return false;
        }

        return true;
    }
    bool YarpClock::terminate(BlockInformation */*S*/, wbt::Error */*error*/)
    {
        yarp::os::Network::fini();
        return true;
    }

    bool YarpClock::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        Signal signal = blockInfo->getOutputPortSignal(0);
        signal.set(0, yarp::os::Time::now());
        return true;
    }
}
