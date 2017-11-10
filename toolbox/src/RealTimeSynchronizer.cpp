#include "RealTimeSynchronizer.h"
#include "Log.h"
#include "BlockInformation.h"

#include <yarp/os/Time.h>

using namespace wbt;

const std::string RealTimeSynchronizer::ClassName = "RealTimeSynchronizer";

const unsigned RealTimeSynchronizer::PARAM_PERIOD = 1; // Period

RealTimeSynchronizer::RealTimeSynchronizer()
: m_period(0.01)
, m_initialTime(0)
, m_counter(0)
{}

unsigned RealTimeSynchronizer::numberOfParameters() { return 1; }

bool RealTimeSynchronizer::configureSizeAndPorts(BlockInformation* blockInfo)
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
    // No outputs
    //

    if (!blockInfo->setNumberOfOutputPorts(0)) {
        Log::getSingleton().error("Failed to set output port number.");
        return false;
    }

    return true;
}

bool RealTimeSynchronizer::initialize(const BlockInformation* blockInfo)
{
    if (!blockInfo->getScalarParameterAtIndex(PARAM_PERIOD, m_period)) {
        Log::getSingleton().error("Failed to get input parametes.");
        return false;
    }

    if (m_period < 0) {
        Log::getSingleton().error("Period must be greater than 0.");
        return false;
    }

    m_counter = 0;
    return true;
}

bool RealTimeSynchronizer::terminate(const BlockInformation* blockInfo)
{
    return true;
}

bool RealTimeSynchronizer::output(const BlockInformation* blockInfo)
{
    if (m_counter == 0) {
        m_initialTime = yarp::os::Time::now();
    }

    //read current time
    double currentTime = yarp::os::Time::now() - m_initialTime;
    double desiredTime = m_counter* m_period;

    double sleepPeriod = desiredTime - currentTime;

    //sleep for the remaining time
    if (sleepPeriod > 0) {
        yarp::os::Time::delay(sleepPeriod);
    }

    m_counter++;

    return true;
}
