#include "MinimumJerkTrajectoryGenerator.h"

#include "Log.h"
#include "BlockInformation.h"
#include "Signal.h"

#include <iCub/ctrl/minJerkCtrl.h>

#include <yarp/os/Network.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <cmath>

namespace wbt {

    const std::string MinimumJerkTrajectoryGenerator::ClassName = "MinimumJerkTrajectoryGenerator";

    const unsigned MinimumJerkTrajectoryGenerator::PARAM_IDX_SAMPLE_TIME   = 1;
    const unsigned MinimumJerkTrajectoryGenerator::PARAM_IDX_SETTLING_TIME = 2;
    const unsigned MinimumJerkTrajectoryGenerator::PARAM_IDX_OUTPUT_1ST_DERIVATIVE = 3;
    const unsigned MinimumJerkTrajectoryGenerator::PARAM_IDX_OUTPUT_2ND_DERIVATIVE = 4;
    const unsigned MinimumJerkTrajectoryGenerator::PARAM_IDX_INITIAL_VALUE    = 5;
    const unsigned MinimumJerkTrajectoryGenerator::PARAM_IDX_EXT_SETTLINGTIME = 6;
    const unsigned MinimumJerkTrajectoryGenerator::PARAM_IDX_RESET_CHANGEST = 7;

    MinimumJerkTrajectoryGenerator::MinimumJerkTrajectoryGenerator()
    : m_generator(nullptr)
    , m_outputFirstDerivativeIndex(-1)
    , m_outputSecondDerivativeIndex(-1)
    , m_firstRun(true)
    , m_explicitInitialValue(false)
    , m_externalSettlingTime(false)
    , m_resetOnSettlingTimeChange(false)
    , m_initialValues(nullptr)
    , m_reference(nullptr)
    {}

    unsigned MinimumJerkTrajectoryGenerator::numberOfParameters() { return 7; }

    bool MinimumJerkTrajectoryGenerator::configureSizeAndPorts(BlockInformation* blockInfo)
    {
        // PARAMETERS
        // ==========
        //
        // 1) Sample time (double)
        // 2) Settling time (double)
        // 3) Enable the output of 1st derivative (bool)
        // 4) Enable the output of 2nd derivative (bool)
        // 5) Enable the input with the initial conditions (bool)
        // 6) Enable the input with the external settling time (bool)
        // 7) Reset the trajectory generator when settling time changes (bool)
        //

        bool outputFirstDerivative;
        bool outputSecondDerivative;
        bool explicitInitialValue;
        bool externalSettlingTime;
        bool ok = true;

        ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_OUTPUT_1ST_DERIVATIVE, outputFirstDerivative);
        ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_OUTPUT_2ND_DERIVATIVE, outputSecondDerivative);
        ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_INITIAL_VALUE, explicitInitialValue);
        ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_EXT_SETTLINGTIME, externalSettlingTime);

        if (!ok) {
            Log::getSingleton().error("Failed to get input parameters.");
            return false;
        }

        int numberOfInputPorts = 1;
        numberOfInputPorts += static_cast<unsigned>(explicitInitialValue);
        numberOfInputPorts += static_cast<unsigned>(externalSettlingTime);

        // INPUTS
        // ======
        //
        // 1) The reference signal (1xn)
        // 2) The optional initial conditions
        // 3) The optional settling time
        //

        if(!blockInfo->setNumberOfInputPorts(numberOfInputPorts)) {
            Log::getSingleton().error("Failed to set input port number.");
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

        if (externalSettlingTime) {
            blockInfo->setInputPortVectorSize(portIndex, 1);
            blockInfo->setInputPortType(portIndex, PortDataTypeDouble);
            portIndex++;
        }

        // OUTPUTS
        // =======
        //
        // 1) The calculated trajectory
        // 2) The optional 1st derivative
        // 3) The optional 2nd derivative
        //

        int numberOfOutputPorts = 1;
        numberOfOutputPorts += static_cast<unsigned>(outputFirstDerivative);
        numberOfOutputPorts += static_cast<unsigned>(outputSecondDerivative);

        if (!blockInfo->setNumberOfOutputPorts(numberOfOutputPorts)) {
            Log::getSingleton().error("Failed to set output port number.");
            return false;
        }

        for (int i = 0; i < numberOfOutputPorts; ++i) {
            blockInfo->setOutputPortVectorSize(i, -1);
            blockInfo->setOutputPortType(i, PortDataTypeDouble);
        }

        return true;
    }

    bool MinimumJerkTrajectoryGenerator::initialize(BlockInformation* blockInfo)
    {
        // Get the additional parameters
        bool outputFirstDerivative;
        bool outputSecondDerivative;
        bool ok = true;

        ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_OUTPUT_1ST_DERIVATIVE,
                                                        outputFirstDerivative);
        ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_OUTPUT_2ND_DERIVATIVE,
                                                        outputSecondDerivative);
        ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_INITIAL_VALUE, m_explicitInitialValue);
        ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_EXT_SETTLINGTIME, m_externalSettlingTime);
        ok = ok & blockInfo->getBooleanParameterAtIndex(PARAM_IDX_RESET_CHANGEST,
                                                        m_resetOnSettlingTimeChange);

        if (!ok) {
            Log::getSingleton().error("Failed to get input parameters.");
            return false;
        }

        if (outputFirstDerivative) {
            m_outputFirstDerivativeIndex = 1;
        }

        if (outputSecondDerivative) {
            m_outputSecondDerivativeIndex = outputFirstDerivative ? 2 : 1;
        }

        double sampleTime;
        double settlingTime;

        ok = ok & blockInfo->getScalarParameterAtIndex(PARAM_IDX_SAMPLE_TIME, sampleTime);
        ok = ok & blockInfo->getScalarParameterAtIndex(PARAM_IDX_SETTLING_TIME, settlingTime);

        unsigned signalSize = blockInfo->getInputPortWidth(0);

        m_generator = new iCub::ctrl::minJerkTrajGen(signalSize, sampleTime, settlingTime);
        m_previousSettlingTime = settlingTime;
        m_initialValues = new yarp::sig::Vector(signalSize);
        m_reference = new yarp::sig::Vector(signalSize);

        if (!m_generator || !m_initialValues || !m_reference) {
            Log::getSingleton().error("Could not allocate memory for trajectory generator.");
            return false;
        }

        m_firstRun = true;
        return true;
    }

    bool MinimumJerkTrajectoryGenerator::terminate(BlockInformation* blockInfo)
    {
        if (m_generator) {
            delete m_generator;
            m_generator = nullptr;
        }
        if (m_initialValues) {
            delete m_initialValues;
            m_initialValues = nullptr;
        }
        if (m_reference) {
            delete m_reference;
            m_reference = nullptr;
        }
        return true;
    }

    bool MinimumJerkTrajectoryGenerator::output(BlockInformation* blockInfo)
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
