#include "Signal.h"

#include <cstring>

namespace wbt {


    Signal::Signal() {}

    void Signal::initSignalType(wbt::PortDataType type, bool constPort)
    {
        this->portType = type;
        this->isConstPort = constPort;
    }

    void Signal::setContiguousBuffer(void* buffer)
    {
        contiguousData = buffer;
        this->isContiguous = true;
    }
    void Signal::setContiguousBuffer(const void* buffer)
    {
        contiguousData = const_cast<void*>(buffer);
        this->isContiguous = true;
    }

    void Signal::setNonContiguousBuffer(void** buffer)
    {
        nonContiguousData = buffer;
        this->isContiguous = false;
    }

    void Signal::setNonContiguousBuffer(const void* const * buffer)
    {
        nonContiguousData = const_cast<void**>(buffer);
        this->isContiguous = false;
    }


    const Data Signal::get(unsigned index) const
    {
        Data data;
        switch (portType) {
            case PortDataTypeDouble:
                if (isContiguous) {
                    data.doubleData((static_cast<double*>(contiguousData))[index]);
                }
                else {
                    const double* buffer = static_cast<const double*>(*nonContiguousData);
                    data.doubleData(static_cast<const double>(buffer[index]));
                }
                break;
            case PortDataTypeSingle:
                if (isContiguous) {
                    data.floatData((static_cast<float*>(contiguousData))[index]);
                }
                else {
                    const float* buffer = static_cast<const float*>(*nonContiguousData);
                    data.floatData(static_cast<const float>(buffer[index]));
                }
                break;
            case PortDataTypeInt8:
                if (isContiguous) {
                    data.int8Data((static_cast<int8_t*>(contiguousData))[index]);
                }
                else {
                    const int8_t* buffer = static_cast<const int8_t*>(*nonContiguousData);
                    data.int8Data(static_cast<const int8_t>(buffer[index]));
                }
                break;
            case PortDataTypeUInt8:
                if (isContiguous) {
                    data.uint8Data((static_cast<uint8_t*>(contiguousData))[index]);
                }
                else {
                    const uint8_t* buffer = static_cast<const uint8_t*>(*nonContiguousData);
                    data.uint8Data(static_cast<const uint8_t>(buffer[index]));
                }
                break;
            case PortDataTypeInt16:
                if (isContiguous) {
                    data.int16Data((static_cast<int16_t*>(contiguousData))[index]);
                }
                else {
                    const int16_t* buffer = static_cast<const int16_t*>(*nonContiguousData);
                    data.int16Data(static_cast<const int16_t>(buffer[index]));
                }
                break;
            case PortDataTypeUInt16:
                if (isContiguous) {
                    data.uint16Data((static_cast<uint16_t*>(contiguousData))[index]);
                }
                else {
                    const uint16_t* buffer = static_cast<const uint16_t*>(*nonContiguousData);
                    data.uint16Data(static_cast<const uint16_t>(buffer[index]));
                }
                break;
            case PortDataTypeInt32:
                if (isContiguous) {
                    data.int32Data((static_cast<int32_t*>(contiguousData))[index]);
                }
                else {
                    const int32_t* buffer = static_cast<const int32_t*>(*nonContiguousData);
                    data.int32Data(static_cast<const int32_t>(buffer[index]));
                }
                break;
            case PortDataTypeUInt32:
                if (isContiguous) {
                    data.uint32Data((static_cast<uint32_t*>(contiguousData))[index]);
                }
                else {
                    const uint32_t* buffer = static_cast<const uint32_t*>(*nonContiguousData);
                    data.uint32Data(static_cast<const uint32_t>(buffer[index]));
                }
                break;
            case PortDataTypeBoolean:
                if (isContiguous) {
                    data.booleanData((static_cast<bool*>(contiguousData))[index]);
                }
                else {
                    const bool* buffer = static_cast<const bool*>(*nonContiguousData);
                    data.booleanData(static_cast<const bool>(buffer[index]));
                }
        }
        return data;
    }

    void* Signal::getContiguousBuffer()
    {
        if (!isContiguous) return nullptr;
        return this->contiguousData;
    }

    //the missing are cast
    void Signal::set(unsigned index, double data)
    {
        if (isConstPort) return;

        switch (portType) {
            case PortDataTypeDouble:
            {
                double *buffer = static_cast<double*>(contiguousData);
                buffer[index] = data;
                break;
            }
            case PortDataTypeSingle:
            {
                float *buffer = static_cast<float*>(contiguousData);
                buffer[index] = data;
                break;
            }
            default:
                break;
        }
    }

    void Signal::setBuffer(const double *data, const unsigned length, unsigned startIndex)
    {
        if (isConstPort) return;
        unsigned dataSize = 0;
        const void * address = data;

        switch (portType) {
            case PortDataTypeDouble:
            {
                dataSize = sizeof(double);
                address = static_cast<const double*>(address) + startIndex;
                break;
            }
            case PortDataTypeSingle:
            {
                dataSize = sizeof(float);
                address = static_cast<const float*>(address) + startIndex;
                break;
            }
            default:
                break;
        }

        memcpy(contiguousData, address, dataSize * length);

    }

    void Signal::set(unsigned index, int32_t data)
    {
        //signed integer function
        switch (portType) {
            case PortDataTypeInt32:
            {
                int32_t *buffer = static_cast<int32_t*>(contiguousData);
                buffer[index] = data;
                break;
            }
            case PortDataTypeInt16:
            {
                int16_t *buffer = static_cast<int16_t*>(contiguousData);
                buffer[index] = data;
                break;
            }
            case PortDataTypeInt8:
            {
                int8_t *buffer = static_cast<int8_t*>(contiguousData);
                buffer[index] = data;
                break;
            }
            default:
                break;
        }
    }

    void Signal::setBuffer(const int32_t *data, const unsigned length, unsigned startIndex)
    {
        if (isConstPort) return;
        unsigned dataSize = 0;
        const void * address = data;

        switch (portType) {
            case PortDataTypeInt32:
            {
                dataSize = sizeof(int32_t);
                address = static_cast<const int32_t*>(address) + startIndex;
                break;
            }
            case PortDataTypeInt16:
            {
                dataSize = sizeof(int16_t);
                address = static_cast<const int16_t*>(address) + startIndex;
                break;
            }
            case PortDataTypeInt8:
            {
                dataSize = sizeof(int8_t);
                address = static_cast<const int8_t*>(address) + startIndex;
                break;
            }
            default:
                break;
        }

        memcpy(contiguousData, address, dataSize * length);
    }

    void Signal::set(unsigned index, uint32_t data)
    {
        //signed integer function
        switch (portType) {
            case PortDataTypeUInt32:
            {
                uint32_t *buffer = static_cast<uint32_t*>(contiguousData);
                buffer[index] = data;
                break;
            }
            case PortDataTypeUInt16:
            {
                uint16_t *buffer = static_cast<uint16_t*>(contiguousData);
                buffer[index] = data;
                break;
            }
            case PortDataTypeUInt8:
            {
                uint8_t *buffer = static_cast<uint8_t*>(contiguousData);
                buffer[index] = data;
                break;
            }
            default:
                break;
        }
    }

    void Signal::setBuffer(const uint32_t *data, const unsigned length, unsigned startIndex)
    {
        if (isConstPort) return;
        unsigned dataSize = 0;
        const void * address = data;

        switch (portType) {
            case PortDataTypeUInt32:
            {
                dataSize = sizeof(uint32_t);
                address = data + startIndex;
                break;
            }
            case PortDataTypeUInt16:
            {
                dataSize = sizeof(uint16_t);
                address = data + startIndex;
                break;
            }
            case PortDataTypeUInt8:
            {
                dataSize = sizeof(uint8_t);
                address = data + startIndex;
                break;
            }
            default:
                break;
        }

        memcpy(contiguousData, address, dataSize * length);
    }

    void Signal::set(unsigned index, bool data)
    {
        bool *buffer = static_cast<bool*>(contiguousData);
        buffer[index] = data;
    }

    void Signal::setBuffer(const bool *data, const unsigned length, unsigned startIndex)
    {
        if (isConstPort) return;
        unsigned dataSize = sizeof(bool);
        const void* address = static_cast<const bool*>(data) + startIndex;

        memcpy(contiguousData, address, dataSize * length);
    }


}
