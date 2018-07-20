/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "MinimumJerkTrajectoryGenerator.h"
#include "Core/BlockInformation.h"
#include "Core/Log.h"
#include "Core/Parameter.h"
#include "Core/Parameters.h"
#include "Core/Signal.h"

#include <iCub/ctrl/minJerkCtrl.h>
#include <yarp/sig/Vector.h>

#include <cmath>
#include <ostream>
#include <tuple>

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

enum InputIndex
{
    InputSignal = 0,
    // Other optional inputs
};

static int InputIndex_InitialValue = InputIndex::InputSignal;
static int InputIndex_ExtSettlingTime = InputIndex::InputSignal;

enum OutputIndex
{
    FilteredSignal = 0,
    // Other optional outputs
};

static int OutputIndex_FirstDer = OutputIndex::FilteredSignal;
static int OutputIndex_SecondDer = OutputIndex::FilteredSignal;

// BLOCK PIMPL
// ===========

class MinimumJerkTrajectoryGenerator::impl
{
public:
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> generator;

    double previousSettlingTime;

    bool firstRun = true;
    bool readInitialValue = false;
    bool readExternalSettlingTime = false;
    bool resetOnSettlingTimeChange = false;
    bool computeFirstDerivative = false;
    bool computeSecondDerivative = false;
    yarp::sig::Vector initialValues;
    yarp::sig::Vector reference;
};

// BLOCK CLASS
// ===========

MinimumJerkTrajectoryGenerator::MinimumJerkTrajectoryGenerator()
    : pImpl{new impl()}
{}

MinimumJerkTrajectoryGenerator::~MinimumJerkTrajectoryGenerator() = default;

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
    // PARAMETERS
    // ==========

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

    // INPUTS
    // ======
    //
    // 1) The reference signal (1xn)
    // 2) The optional initial conditions (1xn)
    // 3) The optional settling time
    //
    // OUTPUTS
    // =======
    //
    // 1) The calculated trajectory (1xn)
    // 2) The optional 1st derivative (1xn)
    // 3) The optional 2nd derivative (1xn)
    //

    // Optional inputs indices
    if (readInitialValue) {
        InputIndex_InitialValue = InputIndex::InputSignal + 1;
    }
    if (readExternalSettlingTime) {
        InputIndex_ExtSettlingTime = InputIndex_InitialValue + 1;
    }

    // Optional output indices
    if (computeFirstDerivative) {
        OutputIndex_FirstDer = OutputIndex::FilteredSignal + 1;
    }
    if (computeSecondDerivative) {
        OutputIndex_SecondDer = OutputIndex_FirstDer + 1;
    }

    BlockInformation::IOData ioData;
    ioData.input.emplace_back(
        InputIndex::InputSignal, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    ioData.output.emplace_back(
        OutputIndex::FilteredSignal, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);

    // Handle optional inputs
    if (readInitialValue) {
        ioData.input.emplace_back(InputIndex_InitialValue, std::vector<int>{1}, DataType::DOUBLE);
    }
    if (readExternalSettlingTime) {
        ioData.input.emplace_back(
            InputIndex_ExtSettlingTime, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    }

    // Handle optional outputs
    if (computeFirstDerivative) {
        ioData.output.emplace_back(
            OutputIndex_FirstDer, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    }
    if (computeSecondDerivative) {
        ioData.output.emplace_back(
            OutputIndex_SecondDer, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    }

    if (!blockInfo->setIOPortsData(ioData)) {
        wbtError << "Failed to configure input / output ports.";
        return false;
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

    bool ok = true;
    ok = ok && m_parameters.getParameter("SampleTime", sampleTime);
    ok = ok && m_parameters.getParameter("SettlingTime", settlingTime);
    ok = ok && m_parameters.getParameter("ComputeFirstDerivative", pImpl->computeFirstDerivative);
    ok = ok && m_parameters.getParameter("ComputeSecondDerivative", pImpl->computeSecondDerivative);
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

    // Get the input port width
    const auto inputPortWidth = blockInfo->getInputPortWidth(InputIndex::InputSignal);

    pImpl->previousSettlingTime = settlingTime;
    pImpl->initialValues.resize(inputPortWidth);
    pImpl->reference.resize(inputPortWidth);
    pImpl->generator = std::unique_ptr<iCub::ctrl::minJerkTrajGen>(
        new iCub::ctrl::minJerkTrajGen(inputPortWidth, sampleTime, settlingTime));

    pImpl->firstRun = true;
    return true;
}

bool MinimumJerkTrajectoryGenerator::output(const BlockInformation* blockInfo)
{
    if (pImpl->readExternalSettlingTime) {
        InputSignalPtr externalTimeSignal =
            blockInfo->getInputPortSignal(InputIndex_ExtSettlingTime);
        if (!externalTimeSignal) {
            wbtError << "Input signal not valid.";
            return false;
        }

        const double externalTime = externalTimeSignal->get<double>(0);

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

        InputSignalPtr initialValuesSignal = blockInfo->getInputPortSignal(InputIndex_InitialValue);
        if (!initialValuesSignal) {
            wbtError << "Input signal not valid.";
            return false;
        }

        for (unsigned i = 0; i < initialValuesSignal->getWidth(); ++i) {
            pImpl->initialValues[i] = initialValuesSignal->get<double>(i);
        }
        pImpl->generator->init(pImpl->initialValues);
    }

    // Input signal
    // ------------

    InputSignalPtr referencesSignal = blockInfo->getInputPortSignal(InputIndex::InputSignal);
    if (!referencesSignal) {
        wbtError << "Input signal not valid.";
        return false;
    }

    for (unsigned i = 0; i < referencesSignal->getWidth(); ++i) {
        pImpl->reference[i] = referencesSignal->get<double>(i);
    }

    pImpl->generator->computeNextValues(pImpl->reference);

    const yarp::sig::Vector& signal = pImpl->generator->getPos();

    // Output signal
    // -------------

    OutputSignalPtr outputSignal = blockInfo->getOutputPortSignal(OutputIndex::FilteredSignal);
    if (!outputSignal) {
        wbtError << "Output signal not valid.";
        return false;
    }

    if (!outputSignal->setBuffer(signal.data(), signal.size())) {
        wbtError << "Failed to set output buffer.";
        return false;
    }

    // First derivative
    // ----------------

    if (pImpl->computeFirstDerivative) {
        const yarp::sig::Vector& derivative = pImpl->generator->getVel();
        OutputSignalPtr firstDerivativeSignal =
            blockInfo->getOutputPortSignal(OutputIndex_FirstDer);
        if (!firstDerivativeSignal) {
            wbtError << "Output signal not valid.";
            return false;
        }

        if (!firstDerivativeSignal->setBuffer(derivative.data(),
                                              firstDerivativeSignal->getWidth())) {
            wbtError << "Failed to set output buffer.";
            return false;
        }
    }

    // Second derivative
    // -----------------

    if (pImpl->computeSecondDerivative) {
        const yarp::sig::Vector& derivative = pImpl->generator->getAcc();
        OutputSignalPtr secondDerivativeSignal =
            blockInfo->getOutputPortSignal(OutputIndex_SecondDer);
        if (!secondDerivativeSignal) {
            wbtError << "Output signal not valid.";
            return false;
        }

        if (!secondDerivativeSignal->setBuffer(derivative.data(),
                                               secondDerivativeSignal->getWidth())) {
            wbtError << "Failed to set output buffer.";
            return false;
        }
    }

    return true;
}
