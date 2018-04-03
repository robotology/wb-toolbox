#include "SimulinkBlockInformation.h"
#include "MxAnyType.h"
#include "Signal.h"

#include <simstruc.h>
#include <string>
#include <vector>

using namespace wbt;

SimulinkBlockInformation::SimulinkBlockInformation(SimStruct* S)
    : simstruct(S)
{}

// BLOCK OPTIONS METHODS
// =====================

bool SimulinkBlockInformation::optionFromKey(const std::string& key, double& option) const
{
    if (key == wbt::BlockOptionPrioritizeOrder) {
        option = SS_OPTION_PLACE_ASAP;
        return true;
    }

    return false;
}

// PARAMETERS METHODS
// ==================

bool SimulinkBlockInformation::getStringParameterAtIndex(unsigned parameterIndex,
                                                         std::string& stringParameter) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
    return MxAnyType(blockParam).asString(stringParameter);
}

bool SimulinkBlockInformation::getScalarParameterAtIndex(unsigned parameterIndex,
                                                         double& value) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
    return MxAnyType(blockParam).asDouble(value);
}

bool SimulinkBlockInformation::getBooleanParameterAtIndex(unsigned parameterIndex,
                                                          bool& value) const
{
    double tmpValue = 0;
    const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);

    // The Simulink mask often doesn't store boolean data from the mask as bool but as double.
    // Calling asBool() will fail in this case. If this happens, asDouble() is used as fallback.
    if (MxAnyType(blockParam).asBool(value)) {
        return true;
    }
    else if (MxAnyType(blockParam).asDouble(tmpValue)) {
        value = static_cast<bool>(tmpValue);
        return true;
    }
    return MxAnyType(blockParam).asBool(value);
}

bool SimulinkBlockInformation::getStructAtIndex(unsigned parameterIndex, AnyStruct& map) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
    return MxAnyType(blockParam).asAnyStruct(map);
}

bool SimulinkBlockInformation::getVectorAtIndex(unsigned parameterIndex,
                                                std::vector<double>& vec) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
    return MxAnyType(blockParam).asVectorDouble(vec);
}

// PORT INFORMATION SETTERS
// ========================

bool SimulinkBlockInformation::setNumberOfInputPorts(const unsigned& numberOfPorts)
{
    return ssSetNumInputPorts(simstruct, numberOfPorts);
}

bool SimulinkBlockInformation::setNumberOfOutputPorts(const unsigned& numberOfPorts)
{
    return ssSetNumOutputPorts(simstruct, numberOfPorts);
}

bool SimulinkBlockInformation::setInputPortVectorSize(const SignalIndex& idx,
                                                      const VectorSize& size)
{
    if (size == Signal::DynamicSize) {
        // TODO: in this case, explore how to use mdlSetOutputPortDimensionInfo and
        // mdlSetDefaultPortDimensionInfo
        return ssSetInputPortVectorDimension(simstruct, idx, DYNAMICALLY_SIZED);
    }

    return ssSetInputPortVectorDimension(simstruct, idx, size);
}

bool SimulinkBlockInformation::setInputPortMatrixSize(const SignalIndex& idx,
                                                      const MatrixSize& size)
{
    // Refer to: https://it.mathworks.com/help/simulink/sfg/sssetoutputportmatrixdimensions.html
    if (size.first == Signal::DynamicSize || size.second == Signal::DynamicSize) {
        // TODO: in this case, explore how to use mdlSetOutputPortDimensionInfo and
        // mdlSetDefaultPortDimensionInfo
        ssSetInputPortMatrixDimensions(simstruct, idx, DYNAMICALLY_SIZED, DYNAMICALLY_SIZED);
    }

    return ssSetInputPortMatrixDimensions(simstruct, idx, size.first, size.first);
}

bool SimulinkBlockInformation::setOutputPortVectorSize(const SignalIndex& idx,
                                                       const VectorSize& size)
{
    if (size == Signal::DynamicSize) {
        // TODO: in this case, explore how to use mdlSetOutputPortDimensionInfo and
        // mdlSetDefaultPortDimensionInfo
        return ssSetOutputPortVectorDimension(simstruct, idx, DYNAMICALLY_SIZED);
    }

    return ssSetOutputPortVectorDimension(simstruct, idx, size);
}

bool SimulinkBlockInformation::setOutputPortMatrixSize(const SignalIndex& idx,
                                                       const MatrixSize& size)
{
    // Refer to: https://it.mathworks.com/help/simulink/sfg/sssetinputportmatrixdimensions.html
    if (size.first == Signal::DynamicSize || size.second == Signal::DynamicSize) {
        // TODO: in this case, explore how to use mdlSetOutputPortDimensionInfo and
        // mdlSetDefaultPortDimensionInfo
        return ssSetOutputPortMatrixDimensions(
            simstruct, idx, DYNAMICALLY_SIZED, DYNAMICALLY_SIZED);
    }

    return ssSetOutputPortMatrixDimensions(simstruct, idx, size.first, size.second);
}

bool SimulinkBlockInformation::setInputPortType(const SignalIndex& idx, const DataType& type)
{
    ssSetInputPortDirectFeedThrough(simstruct, idx, 1);
    ssSetInputPortDataType(simstruct, idx, mapPortTypeToSimulink(type));
    return true;
}

bool SimulinkBlockInformation::setOutputPortType(const SignalIndex& idx, const DataType& type)
{
    ssSetOutputPortDataType(simstruct, idx, mapPortTypeToSimulink(type));
    return true;
}

// PORT INFORMATION GETTERS
// ========================

unsigned SimulinkBlockInformation::getInputPortWidth(const SignalIndex& idx) const
{
    return ssGetInputPortWidth(simstruct, idx);
}

unsigned SimulinkBlockInformation::getOutputPortWidth(const SignalIndex& idx) const
{
    return ssGetOutputPortWidth(simstruct, idx);
}

Signal SimulinkBlockInformation::getInputPortSignal(const SignalIndex& idx,
                                                    const VectorSize& size) const
{
    // Read if the signal is contiguous or non-contiguous
    boolean_T isContiguous = ssGetInputPortRequiredContiguous(simstruct, idx);
    Signal::DataFormat sigDataFormat =
        isContiguous ? Signal::DataFormat::CONTIGUOUS_ZEROCOPY : Signal::DataFormat::NONCONTIGUOUS;

    // Check if the signal is dynamically sized (which means that the dimension
    // cannot be read)
    bool isDynamicallySized = (ssGetInputPortWidth(simstruct, idx) == DYNAMICALLY_SIZED);

    // Note that if the signal is dynamically sized, portWidth is necessary
    if (isDynamicallySized && size == Signal::DynamicSize) {
        wbtError << "Trying to get a dynamically sized signal without specifying its size.";
        return {};
    }

    // Read the width of the signal if it is not provided as input and the signal is not
    // dynamically sized
    VectorSize signalSize = size;
    if (!isDynamicallySized && size == Signal::DynamicSize) {
        signalSize = ssGetInputPortWidth(simstruct, idx);
    }

    // Get the data type of the Signal if set (default: double)
    DTypeId dataType = ssGetInputPortDataType(simstruct, idx);

    switch (sigDataFormat) {
        case Signal::DataFormat::CONTIGUOUS_ZEROCOPY: {
            // Initialize the signal
            bool isConstPort = true;
            Signal signal(Signal::DataFormat::CONTIGUOUS_ZEROCOPY,
                          mapSimulinkToPortType(dataType),
                          isConstPort);
            signal.setWidth(signalSize);
            // Initialize signal's data
            if (!signal.initializeBufferFromContiguousZeroCopy(
                    ssGetInputPortSignal(simstruct, idx))) {
                wbtError << "Failed to inititialize CONTIGUOUS_ZEROCOPY signal at index " << idx
                         << ".";
                return {};
            }
            return signal;
        }
        case Signal::DataFormat::NONCONTIGUOUS: {
            // Initialize the signal
            bool isConstPort = true;
            Signal signal(
                Signal::DataFormat::NONCONTIGUOUS, mapSimulinkToPortType(dataType), isConstPort);
            signal.setWidth(signalSize);
            // Initialize signal's data
            InputPtrsType port = ssGetInputPortSignalPtrs(simstruct, idx);
            if (!signal.initializeBufferFromNonContiguous(static_cast<const void* const*>(port))) {
                wbtError << "Failed to inititialize NONCONTIGUOUS signal at index " << idx << ".";
                return {};
            }
            return signal;
        }
        case Signal::DataFormat::CONTIGUOUS: {
            wbtError << "Failed to inititialize CONTIGUOUS signal at index " << idx << "."
                     << std::endl
                     << "CONTIGUOUS input signals are not yet supported."
                     << "Use CONTIGUOUS_ZEROCOPY instead.";
            return {};
        }
    }
}

wbt::Signal SimulinkBlockInformation::getOutputPortSignal(const SignalIndex& idx,
                                                          const VectorSize& size) const
{
    // Check if the signal is dynamically sized (which means that the dimension
    // cannot be read)
    bool isDynamicallySized = (ssGetOutputPortWidth(simstruct, idx) == DYNAMICALLY_SIZED);

    // Note that if the signal is dynamically sized, portWidth is necessary
    if (isDynamicallySized && size == Signal::DynamicSize) {
        wbtError << "Trying to get a dynamically sized signal without specifying its size.";
        return {};
    }

    // Read the width of the signal if it is not provided as input and the signal is not
    // dynamically sized
    VectorSize signalSize = size;
    if (!isDynamicallySized && size == Signal::DynamicSize) {
        signalSize = ssGetOutputPortWidth(simstruct, idx);
    }

    // Get the data type of the Signal if set (default: double)
    DTypeId dataType = ssGetOutputPortDataType(simstruct, idx);

    bool isConstPort = false;
    Signal signal(
        Signal::DataFormat::CONTIGUOUS_ZEROCOPY, mapSimulinkToPortType(dataType), isConstPort);
    signal.setWidth(signalSize);

    if (!signal.initializeBufferFromContiguousZeroCopy(ssGetOutputPortSignal(simstruct, idx))) {
        wbtError << "Failed to inititialize CONTIGUOUS_ZEROCOPY signal at index " << idx << ".";
        return {};
    }

    return signal;
}

PortDataType SimulinkBlockInformation::mapSimulinkToPortType(const DTypeId& typeId) const
{
    switch (typeId) {
        case SS_DOUBLE:
            return DataType::DOUBLE;
        case SS_SINGLE:
            return DataType::SINGLE;
        case SS_INT8:
            return DataType::INT8;
        case SS_UINT8:
            return DataType::UINT8;
        case SS_INT16:
            return DataType::INT16;
        case SS_UINT16:
            return DataType::UINT16;
        case SS_INT32:
            return DataType::INT32;
        case SS_UINT32:
            return DataType::UINT32;
        case SS_BOOLEAN:
            return DataType::BOOLEAN;
        default:
            return DataType::DOUBLE;
    }
}

DTypeId SimulinkBlockInformation::mapPortTypeToSimulink(const DataType& dataType) const
{
    switch (dataType) {
        case DataType::DOUBLE:
            return SS_DOUBLE;
        case DataType::SINGLE:
            return SS_SINGLE;
        case DataType::INT8:
            return SS_INT8;
        case DataType::UINT8:
            return SS_UINT8;
        case DataType::INT16:
            return SS_INT16;
        case DataType::UINT16:
            return SS_UINT16;
        case DataType::INT32:
            return SS_INT32;
        case DataType::UINT32:
            return SS_UINT32;
        case DataType::BOOLEAN:
            return SS_BOOLEAN;
    }
}
