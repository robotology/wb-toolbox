/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "DiscreteFilter.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"
#include "Signal.h"

#include <iCub/ctrl/filters.h>
#include <yarp/sig/Vector.h>

#include <sstream>
#include <string>
#include <tuple>
#include <vector>

using namespace wbt;
using namespace iCub::ctrl;
using namespace yarp::sig;
const std::string DiscreteFilter::ClassName = "DiscreteFilter";

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = Block::NumberOfParameters - 1,
    FilterStruct,
};

enum InputIndex
{
    InputSignal = 0,
};

enum OutputIndex
{
    FilteredSignal = 0,
};

// BLOCK PIMPL
// ===========

class DiscreteFilter::impl
{
public:
    std::unique_ptr<iCub::ctrl::IFilter> filter;
    yarp::sig::Vector y0;
    yarp::sig::Vector u0;
    yarp::sig::Vector inputSignalVector;
};

// BLOCK CLASS
// ===========

DiscreteFilter::DiscreteFilter()
    : pImpl{new impl()}
{}

unsigned DiscreteFilter::numberOfParameters()
{
    return Block::numberOfParameters() + 1;
}

bool DiscreteFilter::parseParameters(BlockInformation* blockInfo)
{
    const auto& DynPar = ParameterMetadata::DynamicSize;

    const std::vector<ParameterMetadata> metadata{
        {ParameterType::STRUCT_DOUBLE, ParamIndex::FilterStruct, 1, 1, "Fc"},
        {ParameterType::STRUCT_DOUBLE, ParamIndex::FilterStruct, 1, 1, "Ts"},
        {ParameterType::STRUCT_STRING, ParamIndex::FilterStruct, 1, 1, "FilterType"},
        {ParameterType::STRUCT_INT, ParamIndex::FilterStruct, 1, 1, "MedianOrder"},
        {ParameterType::STRUCT_DOUBLE, ParamIndex::FilterStruct, 1, DynPar, "NumCoeffs"},
        {ParameterType::STRUCT_DOUBLE, ParamIndex::FilterStruct, 1, DynPar, "DenCoeffs"},
        {ParameterType::STRUCT_BOOL, ParamIndex::FilterStruct, 1, 1, "InitStatus"},
        {ParameterType::STRUCT_DOUBLE, ParamIndex::FilterStruct, 1, DynPar, "y0"},
        {ParameterType::STRUCT_DOUBLE, ParamIndex::FilterStruct, 1, DynPar, "u0"}};

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            wbtError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool DiscreteFilter::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // INPUTS
    // ======
    //
    // 1) The input signal (1xn)
    //
    // OUTPUTS
    // =======
    //
    // 1) The output signal (1xn)
    //

    const bool ok = blockInfo->setIOPortsData({
        {
            // Inputs
            std::make_tuple(
                InputIndex::InputSignal, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE),
        },
        {
            // Outputs
            std::make_tuple(OutputIndex::FilteredSignal,
                            std::vector<int>{Signal::DynamicSize},
                            DataType::DOUBLE),
        },
    });

    if (!ok) {
        wbtError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool DiscreteFilter::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!DiscreteFilter::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    std::string filter_type;
    std::vector<double> num_coeff;
    std::vector<double> den_coeff;
    std::vector<double> y0;
    std::vector<double> u0;
    double firstOrderLowPassFilter_fc = 0.0;
    double firstOrderLowPassFilter_ts = 0.0;
    int medianFilter_order = 0;
    bool initStatus = false;

    bool ok = true;
    ok = ok && m_parameters.getParameter("FilterType", filter_type);
    ok = ok && m_parameters.getParameter("NumCoeffs", num_coeff);
    ok = ok && m_parameters.getParameter("DenCoeffs", den_coeff);
    ok = ok && m_parameters.getParameter("y0", y0);
    ok = ok && m_parameters.getParameter("u0", u0);
    ok = ok && m_parameters.getParameter("InitStatus", initStatus);
    ok = ok && m_parameters.getParameter("Fc", firstOrderLowPassFilter_fc);
    ok = ok && m_parameters.getParameter("Ts", firstOrderLowPassFilter_ts);
    ok = ok && m_parameters.getParameter("MedianOrder", medianFilter_order);

    if (!ok) {
        wbtError << "Failed to get parameters after their parsing.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    if (filter_type == "Generic") {
        // Check if numerator and denominator are not empty
        if (num_coeff.empty() || den_coeff.empty()) {
            wbtError << "Empty numerator or denominator not allowed.";
            return false;
        }
        // Check if numerator or denominator are scalar and zero
        if ((num_coeff.size() == 1 && num_coeff.front() == 0.0)
            || (den_coeff.size() == 1 && den_coeff.front() == 0.0)) {
            wbtError << "Passed numerator or denominator not valid.";
            return false;
        }
    }

    // Signal sizes
    const auto inputPortWidth = blockInfo->getInputPortWidth(InputIndex::InputSignal);
    const int outputSignalSize = inputPortWidth;

    // Convert the std::vector to yarp::sig::Vector
    yarp::sig::Vector num(num_coeff.size(), num_coeff.data());
    yarp::sig::Vector den(den_coeff.size(), den_coeff.data());

    if (initStatus) {
        // y0 and output signal dimensions should match
        if (y0.size() != outputSignalSize) {
            wbtError << "y0 and output signal sizes don't match.";
            return false;
        }
        // u0 and input signal dimensions should match (used only for Generic)
        if ((filter_type == "Generic") && (u0.size() != inputPortWidth)) {
            wbtError << "(Generic) u0 and input signal sizes don't match.";
            return false;
        }
        // Allocate the initial conditions
        pImpl->y0 = Vector(y0.size(), y0.data());
        pImpl->u0 = Vector(u0.size(), u0.data());
    }
    else {
        // Initialize zero initial conditions
        pImpl->y0.resize(y0.size());
        pImpl->y0.zero();
        pImpl->u0.resize(u0.size());
        pImpl->u0.zero();
    }

    // Allocate the filter object
    // ==========================

    // Generic
    // -------
    if (filter_type == "Generic") {
        pImpl->filter = std::unique_ptr<IFilter>(new Filter(num, den));
    }
    // FirstOrderLowPassFilter
    // -----------------------
    else if (filter_type == "FirstOrderLowPassFilter") {
        if (firstOrderLowPassFilter_fc == 0.0 || firstOrderLowPassFilter_ts == 0.0) {
            wbtError << "(FirstOrderLowPassFilter) You need to "
                        "specify Fc and Ts.";
            return false;
        }
        pImpl->filter = std::unique_ptr<IFilter>(
            new FirstOrderLowPassFilter(firstOrderLowPassFilter_fc, firstOrderLowPassFilter_ts));
    }
    // MedianFilter
    // ------------
    else if (filter_type == "MedianFilter") {
        if (medianFilter_order == 0) {
            wbtError << "(MedianFilter) You need to specify the filter order.";
            return false;
        }
        pImpl->filter = std::unique_ptr<IFilter>(new MedianFilter(medianFilter_order));
    }
    else {
        wbtError << "Filter type not recognized.";
        return false;
    }

    // Initialize buffers
    // ==================

    pImpl->inputSignalVector.resize(inputPortWidth);
    pImpl->inputSignalVector.zero();

    return true;
}

bool DiscreteFilter::initializeInitialConditions(const BlockInformation* /*blockInfo*/)
{
    // Reminder: this function is called when, during runtime, a block is disabled
    // and enabled again. The method ::initialize instead is called just one time.

    // Initialize the filter. This is required because if the signal is not 1D,
    // the default filter constructor initialize a wrongly sized y0.
    // Moreover, the iCub::ctrl::Filter class has an additional constructor that handles
    // the zero-gain case.
    IFilter* baseFilter = pImpl->filter.get();
    Filter* filter_c = dynamic_cast<Filter*>(baseFilter);
    if (filter_c) {
        // The filter is a Filter object
        filter_c->init(pImpl->y0, pImpl->u0);
    }
    else {
        // The filter is not a Filter object
        if (pImpl->filter) {
            pImpl->filter->init(pImpl->y0);
        }
        else {
            wbtError << "Failed to get the IFilter object.";
            return false;
        }
    }

    return true;
}

bool DiscreteFilter::output(const BlockInformation* blockInfo)
{
    if (!pImpl->filter) {
        wbtError << "Failed to retrieve the filter object.";
        return false;
    }

    // Get the input and output signals
    const Signal inputSignal = blockInfo->getInputPortSignal(InputIndex::InputSignal);
    Signal outputSignal = blockInfo->getOutputPortSignal(OutputIndex::FilteredSignal);

    if (!inputSignal.isValid() || !outputSignal.isValid()) {
        wbtError << "Signals not valid.";
        return false;
    }

    // Copy the Signal to the data structure that the filter wants
    for (unsigned i = 0; i < inputSignal.getWidth(); ++i) {
        pImpl->inputSignalVector[i] = inputSignal.get<double>(i);
    }

    // Filter the current component of the input signal
    const Vector& outputVector = pImpl->filter->filt(pImpl->inputSignalVector);

    // Forward the filtered signals to the output port
    if (!outputSignal.setBuffer(outputVector.data(), outputVector.length())) {
        wbtError << "Failed to set output buffer.";
        return false;
    }

    return true;
}
