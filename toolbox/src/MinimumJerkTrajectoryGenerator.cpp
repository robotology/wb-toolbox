/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "MinimumJerkTrajectoryGenerator.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Signal.h"

#include <iCub/ctrl/minJerkCtrl.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Network.h>
#include <yarp/os/Stamp.h>
#include <yarp/sig/Vector.h>

#include <cmath>

using namespace wbt;

const std::string MinimumJerkTrajectoryGenerator::ClassName = "MinimumJerkTrajectoryGenerator";

const unsigned PARAM_IDX_BIAS = Block::NumberOfParameters - 1;
const unsigned PARAM_IDX_SAMPLE_TIME = PARAM_IDX_BIAS + 1;
const unsigned PARAM_IDX_SETTLING_TIME = PARAM_IDX_BIAS + 2;
const unsigned PARAM_IDX_OUTPUT_1ST_DERIVATIVE = PARAM_IDX_BIAS + 3;
const unsigned PARAM_IDX_OUTPUT_2ND_DERIVATIVE = PARAM_IDX_BIAS + 4;
const unsigned PARAM_IDX_INITIAL_VALUE = PARAM_IDX_BIAS + 5;
const unsigned PARAM_IDX_EXT_SETTLINGTIME = PARAM_IDX_BIAS + 6;
const unsigned PARAM_IDX_RESET_CHANGEST = PARAM_IDX_BIAS + 7;

// Cannot use = default due to iCub::ctrl::minJerkTrajGen instantiation
MinimumJerkTrajectoryGenerator::MinimumJerkTrajectoryGenerator() {}

unsigned MinimumJerkTrajectoryGenerator::numberOfParameters()
{
    return Block::numberOfParameters() + 7;
}

bool MinimumJerkTrajectoryGenerator::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata paramMD_sampleTime(PARAM_DOUBLE, PARAM_IDX_SAMPLE_TIME, 1, 1, "SampleTime");

    ParameterMetadata paramMD_settlingTime(
        PARAM_DOUBLE, PARAM_IDX_SETTLING_TIME, 1, 1, "SettlingTime");

    ParameterMetadata paramMD_computeDot(
        PARAM_BOOL, PARAM_IDX_OUTPUT_1ST_DERIVATIVE, 1, 1, "ComputeFirstDerivative");

    ParameterMetadata paramMD_computeDDot(
        PARAM_BOOL, PARAM_IDX_OUTPUT_2ND_DERIVATIVE, 1, 1, "ComputeSecondDerivative");

    ParameterMetadata paramMD_initialValue(
        PARAM_BOOL, PARAM_IDX_INITIAL_VALUE, 1, 1, "ReadInitialValue");

    ParameterMetadata paramMD_extSettlingTime(
        PARAM_BOOL, PARAM_IDX_EXT_SETTLINGTIME, 1, 1, "ReadExternalSettlingTime");

    ParameterMetadata paramMD_resetOnExcSettlingTime(
        PARAM_BOOL, PARAM_IDX_RESET_CHANGEST, 1, 1, "ResetOnSettlingTimeChange");

    bool ok = true;
    ok = ok && blockInfo->addParameterMetadata(paramMD_sampleTime);
    ok = ok && blockInfo->addParameterMetadata(paramMD_settlingTime);
    ok = ok && blockInfo->addParameterMetadata(paramMD_computeDot);
    ok = ok && blockInfo->addParameterMetadata(paramMD_computeDDot);
    ok = ok && blockInfo->addParameterMetadata(paramMD_initialValue);
    ok = ok && blockInfo->addParameterMetadata(paramMD_extSettlingTime);
    ok = ok && blockInfo->addParameterMetadata(paramMD_resetOnExcSettlingTime);

    if (!ok) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool MinimumJerkTrajectoryGenerator::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

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

    if (!parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    bool computeFirstDerivative = false;
    bool computeSecondDerivative = false;
    bool readInitialValue = false;
    bool readExternalSettlingTime = false;

    bool ok = true;
    ok = ok && m_parameters.getParameter("ComputeFirstDerivative", computeFirstDerivative);
    ok = ok && m_parameters.getParameter("ComputeSecondDerivative", computeSecondDerivative);
    ok = ok && m_parameters.getParameter("ReadInitialValue", readInitialValue);
    ok = ok && m_parameters.getParameter("ReadExternalSettlingTime", readExternalSettlingTime);

    if (!ok) {
        wbtError << "Failed to get parameters after their parsing.";
        return false;
    }

    int numberOfInputPorts = 1;
    numberOfInputPorts += static_cast<unsigned>(readInitialValue);
    numberOfInputPorts += static_cast<unsigned>(readExternalSettlingTime);

    // INPUTS
    // ======
    //
    // 1) The reference signal (1xn)
    // 2) The optional initial conditions
    // 3) The optional settling time
    //

    if (!blockInfo->setNumberOfInputPorts(numberOfInputPorts)) {
        wbtError << "Failed to set input port number.";
        return false;
    }

    blockInfo->setInputPortVectorSize(0, Signal::DynamicSize);
    blockInfo->setInputPortType(0, DataType::DOUBLE);

    unsigned portIndex = 1;

    if (readInitialValue) {
        blockInfo->setInputPortVectorSize(portIndex, Signal::DynamicSize);
        blockInfo->setInputPortType(portIndex, DataType::DOUBLE);
        portIndex++;
    }

    if (readExternalSettlingTime) {
        blockInfo->setInputPortVectorSize(portIndex, 1);
        blockInfo->setInputPortType(portIndex, DataType::DOUBLE);
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
    numberOfOutputPorts += static_cast<unsigned>(computeFirstDerivative);
    numberOfOutputPorts += static_cast<unsigned>(computeSecondDerivative);

    if (!blockInfo->setNumberOfOutputPorts(numberOfOutputPorts)) {
        wbtError << "Failed to set output port number.";
        return false;
    }

    for (int i = 0; i < numberOfOutputPorts; ++i) {
        blockInfo->setOutputPortVectorSize(i, Signal::DynamicSize);
        blockInfo->setOutputPortType(i, DataType::DOUBLE);
    }

    return true;
}

bool MinimumJerkTrajectoryGenerator::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    double sampleTime = 0;
    double settlingTime = 0;
    bool computeFirstDerivative = false;
    bool computeSecondDerivative = false;

    bool ok = true;
    ok = ok && m_parameters.getParameter("SampleTime", sampleTime);
    ok = ok && m_parameters.getParameter("SettlingTime", settlingTime);
    ok = ok && m_parameters.getParameter("ComputeFirstDerivative", computeFirstDerivative);
    ok = ok && m_parameters.getParameter("ComputeSecondDerivative", computeSecondDerivative);
    ok = ok && m_parameters.getParameter("ReadInitialValue", m_readInitialValue);
    ok = ok && m_parameters.getParameter("ReadExternalSettlingTime", m_readExternalSettlingTime);
    ok = ok && m_parameters.getParameter("ResetOnSettlingTimeChange", m_resetOnSettlingTimeChange);

    if (!ok) {
        wbtError << "Failed to get parameters after their parsing.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    if (computeFirstDerivative) {
        m_outputFirstDerivativeIndex = 1;
    }

    if (computeSecondDerivative) {
        m_outputSecondDerivativeIndex = computeFirstDerivative ? 2 : 1;
    }

    // Since the signals sizes were set as dynamic in the configureSizeAndPorts,
    // set here the right values into blockInfo
    const unsigned signalSize = blockInfo->getInputPortWidth(0);
    blockInfo->setInputPortVectorSize(0, signalSize);
    blockInfo->setInputPortVectorSize(1, signalSize);
    blockInfo->setOutputPortVectorSize(0, signalSize);
    blockInfo->setOutputPortVectorSize(1, signalSize);
    blockInfo->setOutputPortVectorSize(2, signalSize);

    // Initialize the class
    m_previousSettlingTime = settlingTime;
    m_initialValues = std::unique_ptr<yarp::sig::Vector>(new yarp::sig::Vector(signalSize));
    m_reference = std::unique_ptr<yarp::sig::Vector>(new yarp::sig::Vector(signalSize));
    m_generator = std::unique_ptr<iCub::ctrl::minJerkTrajGen>(
        new iCub::ctrl::minJerkTrajGen(signalSize, sampleTime, settlingTime));

    if (!m_generator || !m_initialValues || !m_reference) {
        wbtError << "Could not allocate memory for trajectory generator.";
        return false;
    }

    m_firstRun = true;
    return true;
}

bool MinimumJerkTrajectoryGenerator::terminate(const BlockInformation* blockInfo)
{
    return true;
}

bool MinimumJerkTrajectoryGenerator::output(const BlockInformation* blockInfo)
{
    if (!m_generator) {
        return false;
    }

    if (m_readExternalSettlingTime) {
        unsigned portIndex = 1;
        if (m_readInitialValue) {
            portIndex++;
        }

        const Signal externalTimeSignal = blockInfo->getInputPortSignal(portIndex);
        if (externalTimeSignal.isValid()) {
            wbtError << "Input signal not valid.";
            return false;
        }

        const double externalTime = externalTimeSignal.get<double>(0);

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
        unsigned portIndex = 0;
        if (m_readInitialValue) {
            portIndex = 1;
        }
        const Signal initialValuesSignal = blockInfo->getInputPortSignal(portIndex);
        if (initialValuesSignal.isValid()) {
            wbtError << "Input signal not valid.";
            return false;
        }

        for (unsigned i = 0; i < initialValuesSignal.getWidth(); ++i) {
            (*m_initialValues)[i] = initialValuesSignal.get<double>(i);
        }
        m_generator->init(*m_initialValues);
    }

    // Input signal
    // ------------

    const Signal referencesSignal = blockInfo->getInputPortSignal(0);
    if (referencesSignal.isValid()) {
        wbtError << "Input signal not valid.";
        return false;
    }

    for (unsigned i = 0; i < referencesSignal.getWidth(); ++i) {
        (*m_reference)[i] = referencesSignal.get<double>(i);
    }

    m_generator->computeNextValues(*m_reference);

    const yarp::sig::Vector& signal = m_generator->getPos();

    // Output signal
    // -------------

    Signal outputSignal = blockInfo->getOutputPortSignal(0);
    if (outputSignal.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }

    outputSignal.setBuffer(signal.data(), signal.size());

    // First derivative
    // ----------------

    if (m_outputFirstDerivativeIndex != -1) {
        const yarp::sig::Vector& derivative = m_generator->getVel();
        Signal firstDerivativeSignal = blockInfo->getOutputPortSignal(m_outputFirstDerivativeIndex);
        if (firstDerivativeSignal.isValid()) {
            wbtError << "Output signal not valid.";
            return false;
        }

        firstDerivativeSignal.setBuffer(derivative.data(), firstDerivativeSignal.getWidth());
    }

    // Second derivative
    // -----------------

    if (m_outputSecondDerivativeIndex != -1) {
        const yarp::sig::Vector& derivative = m_generator->getAcc();
        Signal secondDerivativeSignal =
            blockInfo->getOutputPortSignal(m_outputSecondDerivativeIndex);
        if (secondDerivativeSignal.isValid()) {
            wbtError << "Output signal not valid.";
            return false;
        }

        secondDerivativeSignal.setBuffer(derivative.data(), secondDerivativeSignal.getWidth());
    }

    return true;
}
