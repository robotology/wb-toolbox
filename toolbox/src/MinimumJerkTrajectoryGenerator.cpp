#include "MinimumJerkTrajectoryGenerator.h"
#include "Error.h"

#include <iCub/ctrl/minJerkCtrl.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#define PARAM_IDX_1 1 //Sample Time (double)
#define PARAM_IDX_2 2 //Settling Time (double)
#define PARAM_IDX_3 3 //Output first derivative (boolean)
#define PARAM_IDX_4 4 //Output second derivative (boolean)
#define PARAM_IDX_5 5 //Initial signal value as input (boolean)

#define GET_OPT_SAMPLE_TIME mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_1))
#define GET_OPT_SETTLING_TIME  mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_2))
#define GET_OPT_OUTPUT_1_DERIVATIVE mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_3))
#define GET_OPT_OUTPUT_2_DERIVATIVE mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_4))
#define GET_OPT_EXPLICIT_INITIAL_VALUE mxGetScalar(ssGetSFcnParam(S,PARAM_IDX_5))

namespace wbt {
    
    std::string MinimumJerkTrajectoryGenerator::ClassName = "MinimumJerkTrajectoryGenerator";

    MinimumJerkTrajectoryGenerator::MinimumJerkTrajectoryGenerator()
    : m_generator(0)
    , m_outputFirstDerivativeIndex(-1)
    , m_outputSecondDerivativeIndex(-1)
    , m_firstRun(true) {}
    
    unsigned MinimumJerkTrajectoryGenerator::numberOfParameters() { return 5; }

    bool MinimumJerkTrajectoryGenerator::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        int_T outputFirstDerivative = GET_OPT_OUTPUT_1_DERIVATIVE;
        int_T outputSecondDerivative = GET_OPT_OUTPUT_2_DERIVATIVE;
        int_T explicitInitialValue = GET_OPT_EXPLICIT_INITIAL_VALUE;

        int numberOfInputPorts = explicitInitialValue ? 2 : 1;
        // Specify I/O
        // INPUTS
        if(!ssSetNumInputPorts(S, numberOfInputPorts)) {
            if (error) error->message = "Failed to set input port number";
            return false;
        }

        ssSetInputPortWidth(S, 0, DYNAMICALLY_SIZED);
        ssSetInputPortDataType(S, 0, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough(S, 0, 1);

        if (explicitInitialValue) {
            ssSetInputPortWidth(S, 1, DYNAMICALLY_SIZED);
            ssSetInputPortDataType(S, 1, SS_DOUBLE);
            ssSetInputPortDirectFeedThrough(S, 1, 1);
        }

        // OUTPUTS
        int numberOfOutputPorts = 1;
        if (outputFirstDerivative) numberOfOutputPorts++;
        if (outputSecondDerivative) numberOfOutputPorts++;

        if (!ssSetNumOutputPorts(S, numberOfOutputPorts)) {
            if (error) error->message = "Failed to set output port number";
            return false;
        }

        for (int i = 0; i < numberOfOutputPorts; ++i) {
            ssSetOutputPortWidth(S, i, DYNAMICALLY_SIZED);
            ssSetOutputPortDataType(S, i, SS_DOUBLE);
        }

        return true;
    }

    bool MinimumJerkTrajectoryGenerator::initialize(SimStruct *S, wbt::Error *error)
    {
//        using namespace yarp::os;
//        using namespace yarp::sig;

//        Network::init();
//
//        if (!Network::initialized() || !Network::checkNetwork(5.0)){
//            if (error) error->message = "YARP server wasn't found active!! \n";
//            return false;
//        }

        //Save parameters
        int_T outputFirstDerivative = GET_OPT_OUTPUT_1_DERIVATIVE;
        int_T outputSecondDerivative = GET_OPT_OUTPUT_2_DERIVATIVE;
        if (outputFirstDerivative) m_outputFirstDerivativeIndex = 1;
        if (outputSecondDerivative) {
            m_outputSecondDerivativeIndex = outputFirstDerivative ? 2 : 1;
        }

        double sampleTime = GET_OPT_SAMPLE_TIME;
        double settlingTime = GET_OPT_SETTLING_TIME;

        int_T size = ssGetInputPortWidth(S, 0);

        m_generator = new iCub::ctrl::minJerkTrajGen(size, sampleTime, settlingTime);
        m_initialValues = new yarp::sig::Vector(size);
        m_reference = new yarp::sig::Vector(size);
        if (!m_generator || !m_initialValues || !m_reference) {
            if (error) error->message = "Could not allocate memory for trajectory generator";
            return false;
        }

        m_firstRun = true;
        return true;
    }

    bool MinimumJerkTrajectoryGenerator::terminate(SimStruct *S, wbt::Error *error) 
    {
        if (m_generator) {
            delete m_generator;
            m_generator = 0;
        }
        if (m_initialValues) {
            delete m_initialValues;
            m_initialValues = 0;
        }
        if (m_reference) {
            delete m_reference;
            m_reference = 0;
        }
        return true;
    }
    
    bool MinimumJerkTrajectoryGenerator::output(SimStruct *S, wbt::Error *error) 
    {
        if (!m_generator) return false;

        if (m_firstRun) {
            m_firstRun = false;
            InputRealPtrsType initialValues = 0;
            int portIndex = 0;
            if (ssGetNumInputPorts(S) == 2) {
                portIndex = 1;
            }
            initialValues = ssGetInputPortRealSignalPtrs(S, portIndex);
            for (unsigned i = 0; i < ssGetInputPortWidth(S, portIndex); ++i) {
                (*m_initialValues)[i] = *initialValues[i];
            }
            m_generator->init(*m_initialValues);
        }

        InputRealPtrsType references = ssGetInputPortRealSignalPtrs(S, 0);
        for (unsigned i = 0; i < ssGetInputPortWidth(S, 0); ++i) {
            (*m_reference)[i] = *references[i];
        }
        m_generator->computeNextValues(*m_reference);

        const yarp::sig::Vector &signal = m_generator->getPos();
        real_T *output = ssGetOutputPortRealSignal(S, 0);
        for (unsigned i = 0; i < ssGetOutputPortWidth(S, 0); ++i) {
            output[i] = signal[i];
        }
        //note: index of the port is not known a priori.
        //I should save it in the initialization phase
        if (m_outputFirstDerivativeIndex != -1) {
            const yarp::sig::Vector &derivative = m_generator->getVel();
            real_T *derivativeOutput = ssGetOutputPortRealSignal(S, m_outputFirstDerivativeIndex);
            for (unsigned i = 0; i < ssGetOutputPortWidth(S, m_outputFirstDerivativeIndex); ++i) {
                derivativeOutput[i] = derivative[i];
            }
        }
        if (m_outputSecondDerivativeIndex != -1) {
            const yarp::sig::Vector &derivative = m_generator->getAcc();
            real_T *derivativeOutput = ssGetOutputPortRealSignal(S, m_outputSecondDerivativeIndex);
            for (unsigned i = 0; i < ssGetOutputPortWidth(S, m_outputSecondDerivativeIndex); ++i) {
                derivativeOutput[i] = derivative[i];
            }
        }

        return true;
    }

}