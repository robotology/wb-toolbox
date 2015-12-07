#include "RealTimeSynchronizer.h"
#include "Error.h"

#include <yarp/os/Time.h>

#define PARAM_IDX_1 1                           // Period
#define GET_PERIOD_PARAMETER mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1))

namespace wbt {
    
    std::string RealTimeSynchronizer::ClassName = "RealTimeSynchronizer";

    RealTimeSynchronizer::RealTimeSynchronizer()
    : m_period(0.01)
    , m_initialTime(0)
    , m_counter(0) {}

    RealTimeSynchronizer::~RealTimeSynchronizer() {}
    
    unsigned RealTimeSynchronizer::numberOfParameters() { return 1; }

    bool RealTimeSynchronizer::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        if(!ssSetNumInputPorts(S, 0)) {
            if (error) error->message = "Failed to set input port number to 0";
            return false;
        }
        
        if (!ssSetNumOutputPorts(S, 0)) {
            if (error) error->message = "Failed to set output port number";
            return false;
        }

        return true;
    }

    bool RealTimeSynchronizer::initialize(SimStruct *S, wbt::Error */*error*/)
    {
        m_period = GET_PERIOD_PARAMETER;
        m_counter = 0;
        return m_period > 0;
    }
    
    bool RealTimeSynchronizer::terminate(SimStruct */*S*/, wbt::Error */*error*/)
    {
        return true;
    }
    
    bool RealTimeSynchronizer::output(SimStruct */*S*/, wbt::Error */*error*/)
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
