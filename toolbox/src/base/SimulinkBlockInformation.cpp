#include "SimulinkBlockInformation.h"
#include "Signal.h"
#include "MxAnyType.h"
#include "simstruc.h"
#include <string>
#include <vector>

namespace wbt {

    SimulinkBlockInformation::SimulinkBlockInformation(SimStruct* S)
    : simstruct(S)
    {}

    bool SimulinkBlockInformation::optionFromKey(const std::string& key, double& option) const
    {
        if (key == wbt::BlockOptionPrioritizeOrder) {
            option = SS_OPTION_PLACE_ASAP;
            return true;
        }

        return false;
    }

    //Parameters methods
    bool SimulinkBlockInformation::getStringParameterAtIndex(unsigned parameterIndex, std::string& stringParameter)
    {
        const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
        return MxAnyType(blockParam).asString(stringParameter);
    }

    bool SimulinkBlockInformation::getStructAtIndex(unsigned parameterIndex, AnyStruct& map)
    {
        const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
        return MxAnyType(blockParam).asAnyStruct(map);
    }


    bool SimulinkBlockInformation::getVectorAtIndex(unsigned parameterIndex, std::vector<double>& vec)
    {
        const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
        return MxAnyType(blockParam).asVectorDouble(vec);
    }

    bool SimulinkBlockInformation::getScalarParameterAtIndex(unsigned parameterIndex, double& value)
    {
        const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
        return MxAnyType(blockParam).asDouble(value);
    }

    bool SimulinkBlockInformation::getBooleanParameterAtIndex(unsigned parameterIndex, bool& value)
    {
        const mxArray* blockParam = ssGetSFcnParam(simstruct, parameterIndex);
        return MxAnyType(blockParam).asBool(value);
    }

    //Port information methods
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
        //for now force Direct feedthrough.. If needed create a separate method
        ssSetInputPortDirectFeedThrough(simstruct, portNumber, 1);
        int matlabDataType = -1;
        switch (portType) {
            case PortDataTypeDouble:
                matlabDataType = SS_DOUBLE;
                break;
                case PortDataTypeSingle:
                matlabDataType = SS_SINGLE;
                break;
            case PortDataTypeInt8:
                matlabDataType = SS_INT8;
                break;
            case PortDataTypeUInt8:
                matlabDataType = SS_UINT8;
                break;
            case PortDataTypeInt16:
                matlabDataType = SS_INT16;
                break;
            case PortDataTypeUInt16:
                matlabDataType = SS_UINT16;
                break;
            case PortDataTypeInt32:
                matlabDataType = SS_INT32;
                break;
            case PortDataTypeUInt32:
                matlabDataType = SS_UINT32;
                break;
            case PortDataTypeBoolean:
                matlabDataType = SS_BOOLEAN;
                break;
        }
        return ssSetInputPortDataType(simstruct, portNumber, matlabDataType);
    }

    bool SimulinkBlockInformation::setOutputPortType(unsigned portNumber, PortDataType portType)
    {
        int matlabDataType = -1;
        switch (portType) {
            case PortDataTypeDouble:
                matlabDataType = SS_DOUBLE;
                break;
            case PortDataTypeSingle:
                matlabDataType = SS_SINGLE;
                break;
            case PortDataTypeInt8:
                matlabDataType = SS_INT8;
                break;
            case PortDataTypeUInt8:
                matlabDataType = SS_UINT8;
                break;
            case PortDataTypeInt16:
                matlabDataType = SS_INT16;
                break;
            case PortDataTypeUInt16:
                matlabDataType = SS_UINT16;
                break;
            case PortDataTypeInt32:
                matlabDataType = SS_INT32;
                break;
            case PortDataTypeUInt32:
                matlabDataType = SS_UINT32;
                break;
            case PortDataTypeBoolean:
                matlabDataType = SS_BOOLEAN;
                break;
        }
        return ssSetOutputPortDataType(simstruct, portNumber, matlabDataType);
    }

    //Port data
    unsigned SimulinkBlockInformation::getInputPortWidth(unsigned portNumber)
    {
        return ssGetInputPortWidth(simstruct, portNumber);
    }

    unsigned SimulinkBlockInformation::getOutputPortWidth(unsigned portNumber)
    {
        return ssGetOutputPortWidth(simstruct, portNumber);
    }

    wbt::Signal SimulinkBlockInformation::getInputPortSignal(unsigned portNumber)
    {
        Signal signal;
        InputPtrsType port = ssGetInputPortSignalPtrs(simstruct, portNumber);
        bool isConstPort = true;
        signal.initSignalType(PortDataTypeDouble, isConstPort);
        signal.setNonContiguousBuffer(port);
        return signal;
    }

    wbt::Signal SimulinkBlockInformation::getOutputPortSignal(unsigned portNumber)
    {
        Signal signal;
        bool isConstPort = false;
        signal.initSignalType(PortDataTypeDouble, isConstPort);
        signal.setContiguousBuffer(ssGetOutputPortSignal(simstruct, portNumber));
        return signal;
    }

}
