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
#include "Parameter.h"
#include "Parameters.h"
#include "Signal.h"

#include <iCub/ctrl/minJerkCtrl.h>
#include <yarp/sig/Vector.h>

#include <cmath>
#include <ostream>

using namespace wbt;
const std::string MinimumJerkTrajectoryGenerator::ClassName = "MinimumJerkTrajectoryGenerator";

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = Block::NumberOfParameters - 1,
    SampleTime,
    SettlingTime,
    FirstDerivative,
    SecondDerivative,
    ReadInitialValue,
    ExtSettlingTime,
    ResetOnChange
};
};

// BLOCK PIMPL
// ===========

class MinimumJerkTrajectoryGenerator::impl
{
public:
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> generator;

    int outputFirstDerivativeIndex = -1;
    int outputSecondDerivativeIndex = -1;

    double previousSettlingTime;

    bool firstRun = true;
    bool readInitialValue = false;
    bool readExternalSettlingTime = false;
    bool resetOnSettlingTimeChange = false;
    yarp::sig::Vector initialValues;
    yarp::sig::Vector reference;
};

// BLOCK CLASS
// ===========

MinimumJerkTrajectoryGenerator::MinimumJerkTrajectoryGenerator()
    : pImpl{new impl()}
{}

unsigned MinimumJerkTrajectoryGenerator::numberOfParameters()
{
    return Block::numberOfParameters() + 7;
}

bool MinimumJerkTrajectoryGenerator::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> metadata{
        {ParameterType::DOUBLE, ParamIndex::SampleTime, 1, 1, "SampleTime"},
        {ParameterType::DOUBLE, ParamIndex::SettlingTime, 1, 1, "SettlingTime"},
        {ParameterType::BOOL, ParamIndex::FirstDerivative, 1, 1, "ComputeFirstDerivative"},
        {ParameterType::BOOL, ParamIndex::SecondDerivative, 1, 1, "ComputeSecondDerivative"},
        {ParameterType::BOOL, ParamIndex::ReadInitialValue, 1, 1, "ReadInitialValue"},
        {ParameterType::BOOL, ParamIndex::ExtSettlingTime, 1, 1, "ReadExternalSettlingTime"},
        {ParameterType::BOOL, ParamIndex::ResetOnChange, 1, 1, "ResetOnSettlingTimeChange"}};

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            wbtError << "Failed to store parameter metadata";
            return false;
        }
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

    if (!MinimumJerkTrajectoryGenerator::parseParameters(blockInfo)) {
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

    if (!MinimumJerkTrajectoryGenerator::parseParameters(blockInfo)) {
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
    ok = ok && m_parameters.getParameter("ReadInitialValue", pImpl->readInitialValue);
    ok = ok
         && m_parameters.getParameter("ReadExternalSettlingTime", pImpl->readExternalSettlingTime);
    ok =
        ok
        && m_parameters.getParameter("ResetOnSettlingTimeChange", pImpl->resetOnSettlingTimeChange);

    if (!ok) {
        wbtError << "Failed to get parameters after their parsing.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    if (computeFirstDerivative) {
        pImpl->outputFirstDerivativeIndex = 1;
    }

    if (computeSecondDerivative) {
        pImpl->outputSecondDerivativeIndex = computeFirstDerivative ? 2 : 1;
    }

    // Since the signals sizes were set as dynamic in the configureSizeAndPorts,
    // set here the right values into blockInfo
    const unsigned signalSize = blockInfo->getInputPortWidth(0);
    unsigned numberOfInputPorts = 1 + static_cast<unsigned>(pImpl->readInitialValue)
                                  + static_cast<unsigned>(pImpl->readExternalSettlingTime);
    unsigned numberOfOutputPorts = 1 + static_cast<unsigned>(computeFirstDerivative)
                                   + static_cast<unsigned>(computeSecondDerivative);

    for (unsigned port = 0; port < numberOfInputPorts; ++port) {
        blockInfo->setInputPortVectorSize(port, signalSize);
    }
    for (unsigned port = 0; port < numberOfOutputPorts; ++port) {
        blockInfo->setOutputPortVectorSize(port, signalSize);
    }

    // Initialize the class
    pImpl->previousSettlingTime = settlingTime;
    pImpl->initialValues.resize(signalSize);
    pImpl->reference.resize(signalSize);
    pImpl->generator = std::unique_ptr<iCub::ctrl::minJerkTrajGen>(
        new iCub::ctrl::minJerkTrajGen(signalSize, sampleTime, settlingTime));

    pImpl->firstRun = true;
    return true;
}

bool MinimumJerkTrajectoryGenerator::output(const BlockInformation* blockInfo)
{
    if (pImpl->readExternalSettlingTime) {
        unsigned portIndex = 1;
        if (pImpl->readInitialValue) {
            portIndex++;
        }

        const Signal externalTimeSignal = blockInfo->getInputPortSignal(portIndex);
        if (!externalTimeSignal.isValid()) {
            wbtError << "Input signal not valid.";
            return false;
        }

        const double externalTime = externalTimeSignal.get<double>(0);

        if (std::abs(pImpl->previousSettlingTime - externalTime) > 1e-5) {
            pImpl->previousSettlingTime = externalTime;

            pImpl->generator->setT(externalTime);
            if (pImpl->resetOnSettlingTimeChange && !pImpl->firstRun) {
                pImpl->generator->init(pImpl->generator->getPos());
            }
        }
    }

    if (pImpl->firstRun) {
        pImpl->firstRun = false;
        unsigned portIndex = 0;
        if (pImpl->readInitialValue) {
            portIndex = 1;
        }
        const Signal initialValuesSignal = blockInfo->getInputPortSignal(portIndex);
        if (!initialValuesSignal.isValid()) {
            wbtError << "Input signal not valid.";
            return false;
        }

        for (unsigned i = 0; i < initialValuesSignal.getWidth(); ++i) {
            pImpl->initialValues[i] = initialValuesSignal.get<double>(i);
        }
        pImpl->generator->init(pImpl->initialValues);
    }

    // Input signal
    // ------------

    const Signal referencesSignal = blockInfo->getInputPortSignal(0);
    if (!referencesSignal.isValid()) {
        wbtError << "Input signal not valid.";
        return false;
    }

    for (unsigned i = 0; i < referencesSignal.getWidth(); ++i) {
        pImpl->reference[i] = referencesSignal.get<double>(i);
    }

    pImpl->generator->computeNextValues(pImpl->reference);

    const yarp::sig::Vector& signal = pImpl->generator->getPos();

    // Output signal
    // -------------

    Signal outputSignal = blockInfo->getOutputPortSignal(0);
    if (!outputSignal.isValid()) {
        wbtError << "Output signal not valid.";
        return false;
    }

    if (!outputSignal.setBuffer(signal.data(), signal.size())) {
        wbtError << "Failed to set output buffer.";
    }

    // First derivative
    // ----------------

    if (pImpl->outputFirstDerivativeIndex != -1) {
        const yarp::sig::Vector& derivative = pImpl->generator->getVel();
        Signal firstDerivativeSignal =
            blockInfo->getOutputPortSignal(pImpl->outputFirstDerivativeIndex);
        if (!firstDerivativeSignal.isValid()) {
            wbtError << "Output signal not valid.";
            return false;
        }

        if (!firstDerivativeSignal.setBuffer(derivative.data(), firstDerivativeSignal.getWidth())) {
            wbtError << "Failed to set output buffer.";
        }
    }

    // Second derivative
    // -----------------

    if (pImpl->outputSecondDerivativeIndex != -1) {
        const yarp::sig::Vector& derivative = pImpl->generator->getAcc();
        Signal secondDerivativeSignal =
            blockInfo->getOutputPortSignal(pImpl->outputSecondDerivativeIndex);
        if (!secondDerivativeSignal.isValid()) {
            wbtError << "Output signal not valid.";
            return false;
        }

        if (!secondDerivativeSignal.setBuffer(derivative.data(),
                                              secondDerivativeSignal.getWidth())) {
            wbtError << "Failed to set output buffer.";
        }
    }

    return true;
}
