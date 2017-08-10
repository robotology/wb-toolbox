#include "DiscreteFilter.h"
#include "BlockInformation.h"
#include "Error.h"
#include "ForwardKinematics.h"
#include "Signal.h"
#include <algorithm>
#include <cassert>
#include <iCub/ctrl/filters.h>
#include <sstream>
#include <string>
#include <yarp/sig/Vector.h>

// Parameters
#define PARAM_IDX_FLT_TYPE 1 // ::Filter type
#define PARAM_IDX_NUMCOEFF 2 // ::Filter numerator coefficients
#define PARAM_IDX_DENCOEFF 3 // ::Filter denominator coefficients
#define PARAM_IDX_1LOWP_FC 4 // ::FirstOrderLowPassFilter cut frequency
#define PARAM_IDX_1LOWP_TS 5 // ::FirstOrderLowPassFilter sampling time
#define PARAM_IDX_MD_ORDER 6 // ::MedianFilter order
// Inputs
#define INPUT_IDX_SIGNAL 0
// Outputs
#define OUTPUT_IDX_SIGNAL 0
// Other defines
#define SIGNAL_DYNAMIC_SIZE -1

using namespace wbt;
using namespace iCub::ctrl;
using namespace yarp::sig;

std::string DiscreteFilter::ClassName = "DiscreteFilter";

DiscreteFilter::DiscreteFilter() : firstRun(true), filter(nullptr), inputSignalVector(nullptr)
{
    num = new Vector(0);
    den = new Vector(0);
}

unsigned DiscreteFilter::numberOfParameters()
{
    return 6;
}

bool DiscreteFilter::configureSizeAndPorts(BlockInformation* blockInfo, wbt::Error* error)
{
    // Memory allocation / Saving data not allowed here

    // Specify I/O
    // ===========

    // INPUTS
    // ------

    // Number of input ports
    int numberOfInputPorts = 1;
    if (!blockInfo->setNumberOfInputPorts(numberOfInputPorts)) {
        if (error) {
            error->message = ClassName + " Failed to set input port number.";
        }
        return false;
    }

    // Input port sizes
    blockInfo->setInputPortVectorSize(INPUT_IDX_SIGNAL, SIGNAL_DYNAMIC_SIZE);
    blockInfo->setInputPortType(INPUT_IDX_SIGNAL, PortDataTypeDouble);

    // OUTPUTS
    // -------

    // Number of output ports
    int numberOfOutputPorts = 1;
    if (!blockInfo->setNumberOfOuputPorts(numberOfOutputPorts)) {
        if (error) {
            error->message = ClassName + " Failed to set output port number.";
        }
        return false;
    }

    // Output port sizes
    blockInfo->setOutputPortVectorSize(OUTPUT_IDX_SIGNAL, SIGNAL_DYNAMIC_SIZE);
    blockInfo->setOutputPortType(OUTPUT_IDX_SIGNAL, PortDataTypeDouble);

    return true;
}

bool DiscreteFilter::initialize(BlockInformation* blockInfo, wbt::Error* error)
{
    // Handle the parameters
    // =====================

    // Variables for the filter parameters
    std::string filter_type;
    std::string num_coeff_str;
    std::string den_coeff_str;
    wbt::Data firstOrderLowPassFilter_fc;
    wbt::Data firstOrderLowPassFilter_ts;
    wbt::Data medianFilter_order;

    // Get the scalar parameters
    firstOrderLowPassFilter_fc = blockInfo->getScalarParameterAtIndex(PARAM_IDX_1LOWP_FC);
    firstOrderLowPassFilter_ts = blockInfo->getScalarParameterAtIndex(PARAM_IDX_1LOWP_TS);
    medianFilter_order         = blockInfo->getScalarParameterAtIndex(PARAM_IDX_MD_ORDER);

    // Get the string parameter
    if (!(blockInfo->getStringParameterAtIndex(PARAM_IDX_FLT_TYPE, filter_type)
          && blockInfo->getStringParameterAtIndex(PARAM_IDX_NUMCOEFF, num_coeff_str)
          && blockInfo->getStringParameterAtIndex(PARAM_IDX_DENCOEFF, den_coeff_str))) {
        if (error) {
            error->message = ClassName + " Failed to parse string parameters.";
        }
        return false;
    }

    // Convert the strings from the Matlab syntax ("[1.0 2 3.33]") to yarp::sig::Vector
    stringToYarpVector(num_coeff_str, num);
    stringToYarpVector(den_coeff_str, den);

    // Create the filter object
    // ========================

    // Generic
    // -------
    if (filter_type == "Generic") {
        if (num->length() == 0 || den->length() == 0) {
            if (error) {
                error->message = ClassName + " (Generic) Wrong coefficients size";
            }
            return 1;
        }
        filter = new Filter(*num, *den);
    }
    // FirstOrderLowPassFilter
    // -----------------------
    else if (filter_type == "FirstOrderLowPassFilter") {
        if (firstOrderLowPassFilter_fc.floatData() == 0
            || firstOrderLowPassFilter_ts.floatData() == 0) {
            if (error) {
                error->message = ClassName
                                 + " (FirstOrderLowPassFilter) You need to "
                                   "specify Fc and Ts";
            }
            return false;
        }
        filter = new FirstOrderLowPassFilter(firstOrderLowPassFilter_fc.floatData(),
                                             firstOrderLowPassFilter_ts.floatData());
    }
    // MedianFilter
    // ------------
    else if (filter_type == "MedianFilter") {
        if (medianFilter_order.int32Data() == 0) {
            if (error) {
                error->message = ClassName
                                 + " (MedianFilter) You need to specify the "
                                   "filter order.";
            }
            return false;
        }
        filter = new MedianFilter(medianFilter_order.int32Data());
    }
    else {
        if (error) error->message = ClassName + " Filter type not recognized.";
        return false;
    }

    return true;
}

bool DiscreteFilter::terminate(BlockInformation* blockInfo, wbt::Error* error)
{
    // Deallocate all the memory
    // -------------------------

    if (filter) {
        delete filter;
        filter = nullptr;
    }

    if (num) {
        delete num;
        num = nullptr;
    }

    if (den) {
        delete den;
        den = nullptr;
    }

    if (inputSignalVector) {
        delete inputSignalVector;
        inputSignalVector = nullptr;
    }

    return true;
}

bool DiscreteFilter::output(BlockInformation* blockInfo, wbt::Error* error)
{
    if (filter == nullptr) return false;

    // Get the input and output signals
    Signal inputSignal  = blockInfo->getInputPortSignal(INPUT_IDX_SIGNAL);
    Signal outputSignal = blockInfo->getOutputPortSignal(OUTPUT_IDX_SIGNAL);

    unsigned inputSignalWidth = blockInfo->getInputPortWidth(INPUT_IDX_SIGNAL);

    // Allocate the memory for the input data
    if (firstRun) {
        // Allocate the input signal
        inputSignalVector = new Vector(inputSignalWidth, 0.0);

        // Initialize the filter. This is required because if the signal is not 1D,
        // the default filter constructor initialize a wrongly sized y0
        filter->init(Vector(inputSignalWidth));

        firstRun = false;
    }

    // Copy the Signal to the data structure that the filter wants
    for (unsigned i = 0; i < inputSignalWidth; ++i) {
        (*inputSignalVector)[i] = inputSignal.get(i).doubleData();
    }

    // Filter the current component of the input signal
    const Vector& outputVector = filter->filt(*inputSignalVector);

    // Forward the filtered signals to the output port
    outputSignal.setBuffer(outputVector.data(), outputVector.length());

    return true;
}

void DiscreteFilter::stringToYarpVector(const std::string str, Vector* v)
{
    assert(v != nullptr);

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

    while (sstrm >> f)
        v->push_back(f);
}
