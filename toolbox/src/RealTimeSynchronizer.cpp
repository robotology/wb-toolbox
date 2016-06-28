#include "RealTimeSynchronizer.h"
#include "Error.h"
#include "BlockInformation.h"

#include <yarp/os/Time.h>

#define PARAM_IDX_1 1                           // Period

namespace wbt {
    
    std::string RealTimeSynchronizer::ClassName = "RealTimeSynchronizer";

    RealTimeSynchronizer::RealTimeSynchronizer()
    : m_period(0.01)
    , m_initialTime(0)
    , m_counter(0) {}

    RealTimeSynchronizer::~RealTimeSynchronizer() {}
    
    unsigned RealTimeSynchronizer::numberOfParameters() { return 1; }

    bool RealTimeSynchronizer::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (!blockInfo->setNumberOfInputPorts(0)) {
            if (error) error->message = "Failed to set input port number to 0";
            return false;
        }
        
        if (!blockInfo->setNumberOfOuputPorts(0)) {
            if (error) error->message = "Failed to set output port number";
            return false;
        }

        return true;
    }

    bool RealTimeSynchronizer::initialize(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        m_period = blockInfo->getScalarParameterAtIndex(PARAM_IDX_1).doubleData();
        m_counter = 0;
        return m_period > 0;
    }
    
    bool RealTimeSynchronizer::terminate(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        return true;
    }
    
    bool RealTimeSynchronizer::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        using namespace yarp::os;

        if (m_counter == 0) {
            m_initialTime = yarp::os::Time::now();
        }

        //read current time
        double currentTime = yarp::os::Time::now() - m_initialTime;
        double desiredTime = m_counter * m_period;

        double sleepPeriod = desiredTime - currentTime;

        //sleep for the remaining time
        if (sleepPeriod > 0)
            yarp::os::Time::delay(sleepPeriod);
        
        m_counter++;

        return true;
    }
}
