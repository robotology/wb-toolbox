#include "SimulinkBlockInformation.h"

#include "Signal.h"

#include "simstruc.h"

namespace wbt {

    SimulinkBlockInformation::SimulinkBlockInformation(SimStruct *S)
    : simstruct(S) {}

    SimulinkBlockInformation::~SimulinkBlockInformation() {}

    bool SimulinkBlockInformation::optionFromKey(const std::string& key, Data &option) const
    {
        if (key == wbt::BlockOptionPrioritizeOrder) {
            option.uint32Data(SS_OPTION_PLACE_ASAP);
            return true;
        }

        return false;
    }

    //Parameters methods
    bool SimulinkBlockInformation::getStringParameterAtIndex(unsigned parameterIndex, std::string& stringParameter)
    {
        int_T buflen, status;
        char *buffer = NULL;

        //robot name
        buflen = (1 + mxGetN(ssGetSFcnParam(simstruct, parameterIndex))) * sizeof(mxChar);
        buffer = static_cast<char*>(mxMalloc(buflen));
        status = mxGetString((ssGetSFcnParam(simstruct, parameterIndex)), buffer, buflen);
        if (status) {
            return false;
        }
        stringParameter = buffer;
        mxFree(buffer); buffer = NULL;
        return true;
    }

    Data SimulinkBlockInformation::getScalarParameterAtIndex(unsigned parameterIndex)
    {
        Data data;
        data.doubleData(mxGetScalar(ssGetSFcnParam(simstruct, parameterIndex)));
        return data;
    }

    //Port information methods
    bool SimulinkBlockInformation::setNumberOfInputPorts(unsigned numberOfPorts)
    {
        return ssSetNumInputPorts(simstruct, numberOfPorts);
    }

    bool SimulinkBlockInformation::setNumberOfOuputPorts(unsigned numberOfPorts)
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
        signal.initSignalType(PortDataTypeDouble, true);
        signal.setNonContiguousBuffer(port);
        return signal;
    }

    wbt::Signal SimulinkBlockInformation::getOutputPortSignal(unsigned portNumber)
    {
        Signal signal;
        signal.initSignalType(PortDataTypeDouble, false);
        signal.setContiguousBuffer(ssGetOutputPortSignal(simstruct, portNumber));
        return signal;
    }
    
}
