#ifndef WBT_BLOCKINFORMATION_H
#define WBT_BLOCKINFORMATION_H

#include "AnyType.h"
#include <string>

namespace wbt {
    class BlockInformation;
    class Signal;

    typedef enum _PortDataType {
        PortDataTypeDouble,
        PortDataTypeSingle,
        PortDataTypeInt8,
        PortDataTypeUInt8,
        PortDataTypeInt16,
        PortDataTypeUInt16,
        PortDataTypeInt32,
        PortDataTypeUInt32,
        PortDataTypeBoolean,
    } PortDataType;

    extern const std::string BlockOptionPrioritizeOrder;
}

class wbt::BlockInformation {

public:
    BlockInformation() = default;
    virtual ~BlockInformation() = default;

    // BLOCK OPTIONS METHODS
    // =====================

    /**
     * Convert a block option from its Toolbox identifier to a specific implementation
     *
     * @param [in]  key    identifier of the block option
     * @param [out] option implementation-specific block option
     * @return             true if the option has been converted. False otherwise
     */
    virtual bool optionFromKey(const std::string& key, double& option) const;


    // PARAMETERS METHODS
    // ==================

    /**
     * Reads the parameter at the specified index and interpret it as a string
     *
     * @param parameterIndex         index of the parameter to be read
     * @param [out]stringParameter   resulting parameter
     *
     * @return true if success, false otherwise
     */
    virtual bool getStringParameterAtIndex(unsigned parameterIndex, std::string& stringParameter) const = 0;
    virtual bool getScalarParameterAtIndex(unsigned parameterIndex, double& value) const = 0;
    virtual bool getBooleanParameterAtIndex(unsigned parameterIndex, bool& value) const = 0;
    virtual bool getStructAtIndex(unsigned parameterIndex, AnyStruct& map) const = 0;
    virtual bool getVectorAtIndex(unsigned parameterIndex, std::vector<double>& vec) const = 0;

    // PORT INFORMATION SETTERS
    // ========================

    virtual bool setNumberOfInputPorts(unsigned numberOfPorts) = 0;
    virtual bool setNumberOfOutputPorts(unsigned numberOfPorts) = 0;
    virtual bool setInputPortVectorSize(unsigned portNumber, int portSize) = 0;
    virtual bool setInputPortMatrixSize(unsigned portNumber, int rows, int columns) = 0;
    virtual bool setOutputPortVectorSize(unsigned portNumber, int portSize) = 0;
    virtual bool setOutputPortMatrixSize(unsigned portNumber, int rows, int columns) = 0;

    /**
     * Set data type for the specified input port
     *
     * @param portNumber number of input port
     * @param portType   data type
     *
     * @return true if succeded, false otherwise
     */
    virtual bool setInputPortType(unsigned portNumber, PortDataType portType) = 0;
    virtual bool setOutputPortType(unsigned portNumber, PortDataType portType) = 0;

    // PORT INFORMATION GETTERS
    // ========================

    virtual unsigned getInputPortWidth(unsigned portNumber) const = 0;
    virtual unsigned getOutputPortWidth(unsigned portNumber) const = 0;
    virtual wbt::Signal getInputPortSignal(unsigned portNumber, int portWidth = -1) const = 0;
    virtual wbt::Signal getOutputPortSignal(unsigned portNumber, int portWidth = -1) const  = 0;
};

#endif /* end of include guard: WBT_BLOCKINFORMATION_H */
