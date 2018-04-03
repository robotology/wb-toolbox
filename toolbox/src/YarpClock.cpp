#include "YarpClock.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Signal.h"

#include <yarp/os/Network.h>
#include <yarp/os/Time.h>

using namespace wbt;

const std::string YarpClock::ClassName = "YarpClock";

unsigned YarpClock::numberOfParameters()
{
    return 0;
}

bool YarpClock::configureSizeAndPorts(BlockInformation* blockInfo)
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

    // OUTPUT
    // ======
    //
    // 1) The yarp time. In short, it streams yarp::os::Time::now().
    //

    if (!blockInfo->setNumberOfOutputPorts(1)) {
        wbtError << "Failed to set output port number.";
        return false;
    }

    blockInfo->setOutputPortVectorSize(0, 1);
    blockInfo->setOutputPortType(0, PortDataTypeDouble);

    return true;
}

bool YarpClock::initialize(const BlockInformation* blockInfo)
{
    yarp::os::Network::init();

    if (!yarp::os::Network::initialized() || !yarp::os::Network::checkNetwork(5.0)) {
        wbtError << "YARP server wasn't found active.";
        return false;
    }

    return true;
}

bool YarpClock::terminate(const BlockInformation* /*S*/)
{
    yarp::os::Network::fini();
    return true;
}

bool YarpClock::output(const BlockInformation* blockInfo)
{
    Signal signal = blockInfo->getOutputPortSignal(0);
    signal.set(0, yarp::os::Time::now());
    return true;
}
