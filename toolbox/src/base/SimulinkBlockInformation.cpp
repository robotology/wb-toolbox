#include "SimulinkBlockInformation.h"
#include "Signal.h"
#include "MxAnyType.h"
#include "simstruc.h"
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

bool SimulinkBlockInformation::getStringParameterAtIndex(unsigned parameterIndex, std::string& stringParameter) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
    return MxAnyType(blockParam).asString(stringParameter);
}

bool SimulinkBlockInformation::getScalarParameterAtIndex(unsigned parameterIndex, double& value) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
    return MxAnyType(blockParam).asDouble(value);
}

bool SimulinkBlockInformation::getBooleanParameterAtIndex(unsigned parameterIndex, bool& value) const
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


bool SimulinkBlockInformation::getVectorAtIndex(unsigned parameterIndex, std::vector<double>& vec) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
    return MxAnyType(blockParam).asVectorDouble(vec);
}

// PORT INFORMATION SETTERS
// ========================

bool SimulinkBlockInformation::setNumberOfInputPorts(unsigned numberOfPorts)
{
    return ssSetNumInputPorts(simstruct, numberOfPorts);
}

bool SimulinkBlockInformation::setNumberOfOutputPorts(unsigned numberOfPorts)
{
    return ssSetNumOutputPorts(simstruct, numberOfPorts);
}

bool SimulinkBlockInformation::setInputPortVectorSize(unsigned portNumber, int portSize)
{
    if (portSize == -1) portSize = DYNAMICALLY_SIZED;
    return ssSetInputPortVectorDimension(simstruct,  portNumber, portSize);
}

bool SimulinkBlockInformation::setInputPortMatrixSize(unsigned portNumber, int rows, int columns)
{
    if (rows == -1) rows = DYNAMICALLY_SIZED;
    if (columns == -1) columns = DYNAMICALLY_SIZED;
    return ssSetInputPortMatrixDimensions(simstruct,  portNumber, rows, columns);
}

bool SimulinkBlockInformation::setOutputPortVectorSize(unsigned portNumber, int portSize)
{
    if (portSize == -1) portSize = DYNAMICALLY_SIZED;
    return ssSetOutputPortVectorDimension(simstruct,  portNumber, portSize);
}

bool SimulinkBlockInformation::setOutputPortMatrixSize(unsigned portNumber, int rows, int columns)
{
    if (rows == -1) rows = DYNAMICALLY_SIZED;
    if (columns == -1) columns = DYNAMICALLY_SIZED;
    return ssSetOutputPortMatrixDimensions(simstruct,  portNumber, rows, columns);
}

bool SimulinkBlockInformation::setInputPortType(unsigned portNumber, PortDataType portType)
{
    ssSetInputPortDirectFeedThrough(simstruct, portNumber, 1);
    ssSetInputPortDataType(simstruct, portNumber, mapSimulinkToPortType(portType));
    return true;
}

bool SimulinkBlockInformation::setOutputPortType(unsigned portNumber, PortDataType portType)
{
    ssSetOutputPortDataType(simstruct, portNumber, mapSimulinkToPortType(portType));
    return true;
}

// PORT INFORMATION GETTERS
// ========================

unsigned SimulinkBlockInformation::getInputPortWidth(unsigned portNumber) const
{
    return ssGetInputPortWidth(simstruct, portNumber);
}

unsigned SimulinkBlockInformation::getOutputPortWidth(unsigned portNumber) const
{
    return ssGetOutputPortWidth(simstruct, portNumber);
}

wbt::Signal SimulinkBlockInformation::getInputPortSignal(unsigned portNumber, int portWidth) const
{
    // Read if the signal is contiguous or non-contiguous
    boolean_T isContiguous = ssGetInputPortRequiredContiguous(simstruct, portNumber);
    SignalDataFormat sigDataFormat = isContiguous ? CONTIGUOUS_ZEROCOPY : NONCONTIGUOUS;

    // Check if the signal is dynamically sized (which means that the dimension
    // cannot be read)
    bool isDynamicallySized = (ssGetInputPortWidth(simstruct, portNumber) == DYNAMICALLY_SIZED);

    // Note that if the signal is DYNAMICALLY_SIZED (-1), portWidth is necessary
    if (isDynamicallySized && portWidth == -1) {
        return Signal();
    }

    // Read the width of the signal if it is not provided as input and the signal is not
    // dynamically sized
    if (!isDynamicallySized && portWidth == -1) {
        portWidth = ssGetInputPortWidth(simstruct, portNumber);
    }

    // Get the data type of the Signal if set (default: double)
    DTypeId dataType = ssGetInputPortDataType(simstruct, portNumber);

    switch (sigDataFormat) {
        case CONTIGUOUS_ZEROCOPY: {
            // Initialize the signal
            bool isConstPort = true;
            Signal signal(CONTIGUOUS_ZEROCOPY, mapSimulinkToPortType(dataType), isConstPort);
            signal.setWidth(portWidth);
            // Initialize signal's data
            if (!signal.initializeBufferFromContiguousZeroCopy(ssGetInputPortSignal(simstruct, portNumber))) {
                return Signal();
            }
            return signal;
        }
        case NONCONTIGUOUS: {
            // Initialize the signal
            bool isConstPort = true;
            Signal signal(NONCONTIGUOUS, mapSimulinkToPortType(dataType), isConstPort);
            signal.setWidth(portWidth);
            // Initialize signal's data
            InputPtrsType port = ssGetInputPortSignalPtrs(simstruct, portNumber);
            if (!signal.initializeBufferFromNonContiguous(static_cast<const void* const*>(port))) {
                return Signal();
            }
            return signal;
        }
        default:
            return Signal();
    }
}

wbt::Signal SimulinkBlockInformation::getOutputPortSignal(unsigned portNumber, int portWidth) const
{
    // Check if the signal is dynamically sized (which means that the dimension
    // cannot be read)
    bool isDynamicallySized = (ssGetOutputPortWidth(simstruct, portNumber) == DYNAMICALLY_SIZED);

    // Note that if the signal is DYNAMICALLY_SIZED (-1), portWidth is necessary
    if (isDynamicallySized && portWidth == -1) {
        return Signal();
    }

    // Read the width of the signal if it is not provided as input and the signal is not
    // dynamically sized
    if (!isDynamicallySized && portWidth == -1) {
        portWidth = ssGetOutputPortWidth(simstruct, portNumber);
    }

    // Get the data type of the Signal if set (default: double)
    DTypeId dataType = ssGetOutputPortDataType(simstruct, portNumber);

    bool isConstPort = false;
    Signal signal(CONTIGUOUS_ZEROCOPY, mapSimulinkToPortType(dataType), isConstPort);
    signal.setWidth(portWidth);

    if (!signal.initializeBufferFromContiguousZeroCopy(ssGetOutputPortSignal(simstruct, portNumber))) {
        return Signal();
    }

    return signal;
}

PortDataType SimulinkBlockInformation::mapSimulinkToPortType(const DTypeId& typeId) const
{
    switch (typeId) {
        case SS_DOUBLE:
            return PortDataTypeDouble;
        case SS_SINGLE:
            return PortDataTypeSingle;
        case SS_INT8:
            return PortDataTypeInt8;
        case SS_UINT8:
            return PortDataTypeUInt8;
        case SS_INT16:
            return PortDataTypeInt16;
        case SS_UINT16:
            return PortDataTypeUInt16;
        case SS_INT32:
            return PortDataTypeInt32;
        case SS_UINT32:
            return PortDataTypeUInt32;
        case SS_BOOLEAN:
            return PortDataTypeBoolean;
        default:
            return PortDataTypeDouble;
    }
}

DTypeId SimulinkBlockInformation::mapPortTypeToSimulink(const PortDataType& dataType) const
{
    switch (dataType) {
        case PortDataTypeDouble:
            return SS_DOUBLE;
        case PortDataTypeSingle:
            return SS_SINGLE;
        case PortDataTypeInt8:
            return SS_INT8;
        case PortDataTypeUInt8:
            return SS_UINT8;
        case PortDataTypeInt16:
            return SS_INT16;
        case PortDataTypeUInt16:
            return SS_UINT16;
        case PortDataTypeInt32:
            return SS_INT32;
        case PortDataTypeUInt32:
            return SS_UINT32;
        case PortDataTypeBoolean:
            return SS_BOOLEAN;
    }
}
