#ifndef WBT_BLOCKINFORMATION_H
#define WBT_BLOCKINFORMATION_H

#include "AnyType.h"
#include <string>
#include <cstdint>

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

    class Data
    {
    private:
        double buffer;
    public:
        inline double doubleData() const { return buffer; }
        inline void doubleData(const double& data) { buffer = data; }
        inline float floatData() const { return static_cast<float>(buffer); }
        inline void floatData(const float& data) { buffer = static_cast<float>(data); }

        inline int8_t int8Data() const { return static_cast<int8_t>(buffer); }
        inline void int8Data(const int8_t& data) { buffer = static_cast<int8_t>(data); }
        inline uint8_t uint8Data() const { return static_cast<uint8_t>(buffer); }
        inline void uint8Data(const uint8_t& data) { buffer = static_cast<uint8_t>(data); }

        inline int16_t int16Data() const { return static_cast<int16_t>(buffer); }
        inline void int16Data(const int16_t& data) { buffer = static_cast<int16_t>(data); }
        inline uint16_t uint16Data() const { return static_cast<uint16_t>(buffer); }
        inline void uint16Data(const uint16_t& data) { buffer = static_cast<uint16_t>(data); }

        inline int32_t int32Data() const { return static_cast<int32_t>(buffer); }
        inline void int32Data(const int32_t& data) { buffer = static_cast<int32_t>(data); }
        inline uint32_t uint32Data() const { return static_cast<uint32_t>(buffer); }
        inline void uint32Data(const uint32_t& data) { buffer = static_cast<uint32_t>(data); }

        inline bool booleanData() const { return static_cast<bool>(buffer); }
        inline void booleanData(const bool& data) { buffer = static_cast<bool>(data); }

        friend Signal;
    };

    extern const std::string BlockOptionPrioritizeOrder;
}

class wbt::BlockInformation {

public:
    virtual ~BlockInformation();

    // Block Options methods
    // =====================

    /**
     * Convert a block option from its Toolbox identifier to a specific implementation
     *
     * @param [in]  key    identifier of the block option
     * @param [out] option implementation-specific block option
     * @return             true if the option has been converted. False otherwise
     */
    virtual bool optionFromKey(const std::string& key, double& option) const;


    // Parameters methods
    // ==================

    /**
     * Reads the parameter at the specified index and interpret it as a string
     *
     * @param parameterIndex         index of the parameter to be read
     * @param [out]stringParameter   resulting parameter
     *
     * @return true if success, false otherwise
     */
    virtual bool getStringParameterAtIndex(unsigned parameterIndex, std::string& stringParameter) = 0;
    virtual bool getScalarParameterAtIndex(unsigned parameterIndex, double& value) = 0;
    virtual bool getBooleanParameterAtIndex(unsigned parameterIndex, bool& value) = 0;
    virtual bool getStructAtIndex(unsigned parameterIndex, AnyStruct& map) = 0;
    virtual bool getVectorAtIndex(unsigned parameterIndex, std::vector<double>& vec) = 0;

    // Port information methods
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

    // Port data
    // =========

    virtual unsigned getInputPortWidth(unsigned portNumber) = 0;
    virtual unsigned getOutputPortWidth(unsigned portNumber) = 0;
    virtual wbt::Signal getInputPortSignal(unsigned portNumber) = 0;
    virtual wbt::Signal getOutputPortSignal(unsigned portNumber) = 0;
};

#endif /* end of include guard: WBT_BLOCKINFORMATION_H */
