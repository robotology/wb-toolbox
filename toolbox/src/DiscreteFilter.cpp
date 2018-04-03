#include "DiscreteFilter.h"
#include "BlockInformation.h"
#include "Log.h"
#include "Signal.h"

#include <iCub/ctrl/filters.h>
#include <yarp/sig/Vector.h>

#include <algorithm>
#include <cassert>
#include <sstream>
#include <string>

using namespace wbt;
using namespace iCub::ctrl;
using namespace yarp::sig;

const std::string DiscreteFilter::ClassName = "DiscreteFilter";

// Parameters
const unsigned DiscreteFilter::PARAM_IDX_FLT_TYPE = 1; // ::Filter type
const unsigned DiscreteFilter::PARAM_IDX_NUMCOEFF = 2; // ::Filter numerator coefficients
const unsigned DiscreteFilter::PARAM_IDX_DENCOEFF = 3; // ::Filter denominator coefficients
const unsigned DiscreteFilter::PARAM_IDX_1LOWP_FC = 4; // ::FirstOrderLowPassFilter cut frequency
const unsigned DiscreteFilter::PARAM_IDX_1LOWP_TS = 5; // ::FirstOrderLowPassFilter sampling time
const unsigned DiscreteFilter::PARAM_IDX_MD_ORDER = 6; // ::MedianFilter order
const unsigned DiscreteFilter::PARAM_IDX_INIT_Y0 = 7; // Output initialization
const unsigned DiscreteFilter::PARAM_IDX_INIT_U0 = 8; // ::Filter input initialization

// Inputs
const unsigned DiscreteFilter::INPUT_IDX_SIGNAL = 0;
// Outputs
const unsigned DiscreteFilter::OUTPUT_IDX_SIGNAL = 0;
// Other defines
const int DiscreteFilter::SIGNAL_DYNAMIC_SIZE = -1;

DiscreteFilter::DiscreteFilter() {}

unsigned DiscreteFilter::numberOfParameters()
{
    return 8;
}

bool DiscreteFilter::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // Memory allocation / Saving data not allowed here

    // Specify I/O
    // ===========

    // INPUTS
    // ------

    // Number of input ports
    int numberOfInputPorts = 1;
    if (!blockInfo->setNumberOfInputPorts(numberOfInputPorts)) {
        wbtError << "Failed to set input port number.";
        return false;
    }

    // Input port sizes
    blockInfo->setInputPortVectorSize(INPUT_IDX_SIGNAL, SIGNAL_DYNAMIC_SIZE);
    blockInfo->setInputPortType(INPUT_IDX_SIGNAL, PortDataTypeDouble);

    // OUTPUTS
    // -------

    // Number of output ports
    int numberOfOutputPorts = 1;
    if (!blockInfo->setNumberOfOutputPorts(numberOfOutputPorts)) {
        wbtError << "Failed to set output port number.";
        return false;
    }

    // Output port sizes
    blockInfo->setOutputPortVectorSize(OUTPUT_IDX_SIGNAL, SIGNAL_DYNAMIC_SIZE);
    blockInfo->setOutputPortType(OUTPUT_IDX_SIGNAL, PortDataTypeDouble);

    return true;
}

bool DiscreteFilter::initialize(const BlockInformation* blockInfo)
{
    // Handle the parameters
    // =====================

    // Variables for the filter parameters
    std::string filter_type;
    std::string num_coeff_str;
    std::string den_coeff_str;
    std::string y0_str;
    std::string u0_str;
    double firstOrderLowPassFilter_fc;
    double firstOrderLowPassFilter_ts;
    double medianFilter_order;

    // Get the scalar parameters
    bool ok = true;
    ok = ok && blockInfo->getScalarParameterAtIndex(PARAM_IDX_1LOWP_FC, firstOrderLowPassFilter_fc);
    ok = ok && blockInfo->getScalarParameterAtIndex(PARAM_IDX_1LOWP_TS, firstOrderLowPassFilter_ts);
    ok = ok && blockInfo->getScalarParameterAtIndex(PARAM_IDX_MD_ORDER, medianFilter_order);

    // Get the string parameter
    ok = ok && blockInfo->getStringParameterAtIndex(PARAM_IDX_FLT_TYPE, filter_type);
    ok = ok && blockInfo->getStringParameterAtIndex(PARAM_IDX_NUMCOEFF, num_coeff_str);
    ok = ok && blockInfo->getStringParameterAtIndex(PARAM_IDX_DENCOEFF, den_coeff_str);
    ok = ok && blockInfo->getStringParameterAtIndex(PARAM_IDX_INIT_Y0, y0_str);
    ok = ok && blockInfo->getStringParameterAtIndex(PARAM_IDX_INIT_U0, u0_str);

    if (!ok) {
        wbtError << "Failed to get parameters after their parsing.";
        return false;
    }

    // Convert the strings from the Matlab syntax ("[1.0 2 3.33]") to yarp::sig::Vector
    yarp::sig::Vector num(0);
    yarp::sig::Vector den(0);
    stringToYarpVector(num_coeff_str, num);
    stringToYarpVector(den_coeff_str, den);

    // y0 and u0 are none if they are not defined
    unsigned y0Width, u0Width;

    if (y0_str != "none") {
        y0 = std::unique_ptr<yarp::sig::Vector>(new Vector(0));
        stringToYarpVector(y0_str, *y0);
        y0Width = y0->length();
    }

    if (u0_str != "none") {
        u0 = std::unique_ptr<yarp::sig::Vector>(new Vector(0));
        stringToYarpVector(u0_str, *u0);
        u0Width = u0->length();
    }

    // Create the filter object
    // ========================

    // Generic
    // -------
    if (filter_type == "Generic") {
        if (num.length() == 0 || den.length() == 0) {
            Log::getSingleton().error("(Generic) Wrong coefficients size.");
            return false;
        }
        filter = std::unique_ptr<Filter>(new Filter(num, den));
    }
    // FirstOrderLowPassFilter
    // -----------------------
    else if (filter_type == "FirstOrderLowPassFilter") {
        if (firstOrderLowPassFilter_fc == 0 || firstOrderLowPassFilter_ts == 0) {
            wbtError << "(FirstOrderLowPassFilter) You need to "
                        "specify Fc and Ts.";
            return false;
        }
        filter = std::unique_ptr<FirstOrderLowPassFilter>(
            new FirstOrderLowPassFilter(firstOrderLowPassFilter_fc, firstOrderLowPassFilter_ts));
    }
    // MedianFilter
    // ------------
    else if (filter_type == "MedianFilter") {
        if (static_cast<int>(medianFilter_order) == 0) {
            wbtError << "(MedianFilter) You need to specify the filter order.";
            return false;
        }
        filter =
            std::unique_ptr<MedianFilter>(new MedianFilter(static_cast<int>(medianFilter_order)));
    }
    else {
        wbtError << "Filter type not recognized.";
        return false;
    }

    // Initialize the other data
    // =========================

    // Get the width of the input vector
    inputSignalWidth = blockInfo->getInputPortWidth(INPUT_IDX_SIGNAL);

    // Check the initial conditions are properly sized
    unsigned outputSignalWidth = blockInfo->getInputPortWidth(OUTPUT_IDX_SIGNAL);

    if ((y0 != nullptr) && (y0Width != outputSignalWidth)) {
        Log::getSingleton().error("y0 and output signal sizes don't match.");
        return false;
    }

    if ((u0 != nullptr) && (filter_type == "Generic") && (u0Width != inputSignalWidth)) {
        Log::getSingleton().error("(Generic) u0 and input signal sizes don't match.");
        return false;
    }

    // Allocate the input signal
    inputSignalVector = std::unique_ptr<Vector>(new Vector(inputSignalWidth, 0.0));

    return true;
}

bool DiscreteFilter::terminate(const BlockInformation* blockInfo)
{
    return true;
}

bool DiscreteFilter::initializeInitialConditions(const BlockInformation* blockInfo)
{
    // Reminder: this function is called when, during runtime, a block is disabled
    // and enabled again. The method ::initialize instead is called just one time.

    // If the initial conditions for the output are not set, allocate a properly
    // sized vector
    if (y0 == nullptr) {
        unsigned outputSignalWidth = blockInfo->getInputPortWidth(OUTPUT_IDX_SIGNAL);
        y0 = std::unique_ptr<Vector>(new Vector(outputSignalWidth, 0.0));
    }
    if (u0 == nullptr) {
        u0 = std::unique_ptr<Vector>(new Vector(inputSignalWidth, 0.0));
    }

    // Initialize the filter. This is required because if the signal is not 1D,
    // the default filter constructor initialize a wrongly sized y0
    // Moreover, the Filter class has a different constructor that handles the
    // zero-gain case
    Filter* filter_c = dynamic_cast<Filter*>(filter.get());
    if (filter_c != nullptr) {
        filter_c->init(*y0, *u0);
    }
    else {
        filter->init(*y0);
    }

    return true;
}

bool DiscreteFilter::output(const BlockInformation* blockInfo)
{
    if (filter == nullptr) {
        return false;
    }

    // Get the input and output signals
    Signal inputSignal = blockInfo->getInputPortSignal(INPUT_IDX_SIGNAL);
    Signal outputSignal = blockInfo->getOutputPortSignal(OUTPUT_IDX_SIGNAL);

    // Copy the Signal to the data structure that the filter wants
    for (unsigned i = 0; i < inputSignalWidth; ++i) {
        (*inputSignalVector)[i] = inputSignal.get<double>(i);
    }

    // Filter the current component of the input signal
    const Vector& outputVector = filter->filt(*inputSignalVector);

    // Forward the filtered signals to the output port
    outputSignal.setBuffer(outputVector.data(), outputVector.length());

    return true;
}

void DiscreteFilter::stringToYarpVector(const std::string str, Vector& v)
{
    std::string s = str;

    // Lambda expression used to remove "[]," carachters
    // TODO: what about removing everything but digits and "."?
    auto lambda_remove_chars = [](const char& c) {
        if ((c == '[') || (c == ']') || c == ',')
            return true;
        else
            return false;
    };

    // Apply the lambda expression the input parameters
    s.erase(remove_if(s.begin(), s.end(), lambda_remove_chars), s.end());

    // Convert the cleaned string to a yarp vector of floats
    std::istringstream sstrm(s);
    float f;

    while (sstrm >> f) {
        v.push_back(f);
    }
}
