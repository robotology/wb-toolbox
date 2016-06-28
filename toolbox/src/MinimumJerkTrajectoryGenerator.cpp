#include "MinimumJerkTrajectoryGenerator.h"

#include "Error.h"
#include "BlockInformation.h"
#include "Signal.h"

#include <iCub/ctrl/minJerkCtrl.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <cmath>

#define PARAM_IDX_1 1 //Sample Time (double)
#define PARAM_IDX_2 2 //Settling Time (double)
#define PARAM_IDX_3 3 //Output first derivative (boolean)
#define PARAM_IDX_4 4 //Output second derivative (boolean)
#define PARAM_IDX_5 5 //Initial signal value as input (boolean)
#define PARAM_IDX_6 6 //Control if the settling time comes from external port or static parameter
#define PARAM_IDX_7 7 //True if the block should reset the traj generator in case settling time changes

namespace wbt {
    
    std::string MinimumJerkTrajectoryGenerator::ClassName = "MinimumJerkTrajectoryGenerator";

    MinimumJerkTrajectoryGenerator::MinimumJerkTrajectoryGenerator()
    : m_generator(0)
    , m_outputFirstDerivativeIndex(-1)
    , m_outputSecondDerivativeIndex(-1)
    , m_firstRun(true)
    , m_explicitInitialValue(false)
    , m_externalSettlingTime(false)
    , m_resetOnSettlingTimeChange(false)
    , m_initialValues(0)
    , m_reference(0) {}
    
    unsigned MinimumJerkTrajectoryGenerator::numberOfParameters() { return 7; }

    bool MinimumJerkTrajectoryGenerator::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        bool outputFirstDerivative = blockInfo->getScalarParameterAtIndex(PARAM_IDX_3).booleanData();
        bool outputSecondDerivative = blockInfo->getScalarParameterAtIndex(PARAM_IDX_4).booleanData();
        bool explicitInitialValue = blockInfo->getScalarParameterAtIndex(PARAM_IDX_5).booleanData();
        bool externalSettlingTime = blockInfo->getScalarParameterAtIndex(PARAM_IDX_6).booleanData();

        int numberOfInputPorts = 1;
        if (explicitInitialValue)
            numberOfInputPorts++;
        if (externalSettlingTime)
            numberOfInputPorts++;

        // Specify I/O
        // INPUTS
        if(!blockInfo->setNumberOfInputPorts(numberOfInputPorts)) {
            if (error) error->message = "Failed to set input port number";
            return false;
        }

        blockInfo->setInputPortVectorSize(0, -1);
        blockInfo->setInputPortType(0, PortDataTypeDouble);

        unsigned portIndex = 1;

        if (explicitInitialValue) {
            blockInfo->setInputPortVectorSize(portIndex, -1);
            blockInfo->setInputPortType(portIndex, PortDataTypeDouble);
            portIndex++;
        }

        if (externalSettlingTime)
        {
            blockInfo->setInputPortVectorSize(portIndex, 1);
            blockInfo->setInputPortType(portIndex, PortDataTypeDouble);
            portIndex++;
        }

        // OUTPUTS
        int numberOfOutputPorts = 1;
        if (outputFirstDerivative) numberOfOutputPorts++;
        if (outputSecondDerivative) numberOfOutputPorts++;

        if (!blockInfo->setNumberOfOuputPorts(numberOfOutputPorts)) {
            if (error) error->message = "Failed to set output port number";
            return false;
        }

        for (int i = 0; i < numberOfOutputPorts; ++i) {
            blockInfo->setOutputPortVectorSize(i, -1);
            blockInfo->setOutputPortType(i, PortDataTypeDouble);
        }

        return true;
    }

    bool MinimumJerkTrajectoryGenerator::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        //Save parameters
        bool outputFirstDerivative = blockInfo->getScalarParameterAtIndex(PARAM_IDX_3).booleanData();
        bool outputSecondDerivative = blockInfo->getScalarParameterAtIndex(PARAM_IDX_4).booleanData();

        if (outputFirstDerivative) m_outputFirstDerivativeIndex = 1;
        if (outputSecondDerivative) {
            m_outputSecondDerivativeIndex = outputFirstDerivative ? 2 : 1;
        }

        double sampleTime = blockInfo->getScalarParameterAtIndex(PARAM_IDX_1).doubleData();
        double settlingTime = blockInfo->getScalarParameterAtIndex(PARAM_IDX_2).doubleData();

        m_explicitInitialValue = blockInfo->getScalarParameterAtIndex(PARAM_IDX_5).booleanData();
        m_externalSettlingTime = blockInfo->getScalarParameterAtIndex(PARAM_IDX_6).booleanData();
        m_resetOnSettlingTimeChange = blockInfo->getScalarParameterAtIndex(PARAM_IDX_7).booleanData();

        unsigned size = blockInfo->getInputPortWidth(0);

        m_generator = new iCub::ctrl::minJerkTrajGen(size, sampleTime, settlingTime);
        m_previousSettlingTime = settlingTime;
        m_initialValues = new yarp::sig::Vector(size);
        m_reference = new yarp::sig::Vector(size);
        if (!m_generator || !m_initialValues || !m_reference) {
            if (error) error->message = "Could not allocate memory for trajectory generator";
            return false;
        }

        m_firstRun = true;
        return true;
    }

    bool MinimumJerkTrajectoryGenerator::terminate(BlockInformation *blockInfo, wbt::Error *error) 
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
    
    bool MinimumJerkTrajectoryGenerator::output(BlockInformation *blockInfo, wbt::Error *error) 
    {
        if (!m_generator) return false;

        if (m_externalSettlingTime) {
            unsigned portIndex = 1;
            if (m_explicitInitialValue) portIndex++;
            Signal externalTimePort = blockInfo->getInputPortSignal(portIndex);
            double externalTime = externalTimePort.get(0).doubleData();

            if (std::abs(m_previousSettlingTime - externalTime) > 1e-5) {
                m_previousSettlingTime = externalTime;

                m_generator->setT(externalTime);
                if (m_resetOnSettlingTimeChange && !m_firstRun) {
                    m_generator->init(m_generator->getPos());
                }
            }
        }

        if (m_firstRun) {
            m_firstRun = false;
            Signal initialValues;
            unsigned portIndex = 0;
            if (m_explicitInitialValue) {
                portIndex = 1;
            }
            initialValues = blockInfo->getInputPortSignal(portIndex);
            for (unsigned i = 0; i < blockInfo->getInputPortWidth(portIndex); ++i) {
                (*m_initialValues)[i] = initialValues.get(i).doubleData();
            }
            m_generator->init(*m_initialValues);
        }


        Signal references = blockInfo->getInputPortSignal(0);
        for (unsigned i = 0; i < blockInfo->getInputPortWidth(0); ++i) {
            (*m_reference)[i] = references.get(i).doubleData();
        }
        m_generator->computeNextValues(*m_reference);

        const yarp::sig::Vector &signal = m_generator->getPos();
        Signal output = blockInfo->getOutputPortSignal(0);
        output.setBuffer(signal.data(), signal.size());
//        for (unsigned i = 0; i < blockInfo->getOutputPortWidth(0); ++i) {
//            output.set(i, signal[i]);
//        }
        //note: index of the port is not known a priori.
        //I should save it in the initialization phase
        if (m_outputFirstDerivativeIndex != -1) {
            const yarp::sig::Vector &derivative = m_generator->getVel();
            Signal derivativeOutput = blockInfo->getOutputPortSignal(m_outputFirstDerivativeIndex);
            derivativeOutput.setBuffer(derivative.data(), blockInfo->getOutputPortWidth(m_outputFirstDerivativeIndex));
        }
        if (m_outputSecondDerivativeIndex != -1) {
            const yarp::sig::Vector &derivative = m_generator->getAcc();
            Signal derivativeOutput = blockInfo->getOutputPortSignal(m_outputSecondDerivativeIndex);
            derivativeOutput.setBuffer(derivative.data(), blockInfo->getOutputPortWidth(m_outputSecondDerivativeIndex));
        }

        return true;
    }

}
