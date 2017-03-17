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
                data.doubleData(isContiguous ? ((double*)contiguousData)[index] : *((double **)nonContiguousData)[index]);
                break;
            case PortDataTypeSingle:
                data.floatData(isContiguous ? ((float*)contiguousData)[index] : *((float **)nonContiguousData)[index]);
                break;
            case PortDataTypeInt8:
                data.int8Data(isContiguous ? ((int8_t*)contiguousData)[index] : *((int8_t **)nonContiguousData)[index]);
                break;
            case PortDataTypeUInt8:
                data.uint8Data(isContiguous ? ((uint8_t*)contiguousData)[index] : *((uint8_t **)nonContiguousData)[index]);
                break;
            case PortDataTypeInt16:
                data.int8Data(isContiguous ? ((int16_t*)contiguousData)[index] : *((int16_t **)nonContiguousData)[index]);
                break;
            case PortDataTypeUInt16:
                data.uint8Data(isContiguous ? ((uint16_t*)contiguousData)[index] : *((uint16_t **)nonContiguousData)[index]);
                break;
            case PortDataTypeInt32:
                data.int8Data(isContiguous ? ((int32_t*)contiguousData)[index] : *((int32_t **)nonContiguousData)[index]);
                break;
            case PortDataTypeUInt32:
                data.uint8Data(isContiguous ? ((uint32_t*)contiguousData)[index] : *((uint32_t **)nonContiguousData)[index]);
                break;
            case PortDataTypeBoolean:
                data.booleanData(isContiguous ? ((bool*)contiguousData)[index] : *((bool **)nonContiguousData)[index]);
        }
        return data;
    }

    void* Signal::getContiguousBuffer()
    {
        if (!isContiguous) return 0;
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
                address = data + startIndex;
                break;
            }
            case PortDataTypeSingle:
            {
                dataSize = sizeof(float);
                address = ((float*)data) + startIndex;
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
                address = ((int32_t*)data) + startIndex;
                break;
            }
            case PortDataTypeInt16:
            {
                dataSize = sizeof(int16_t);
                address = ((int16_t*)data) + startIndex;
                break;
            }
            case PortDataTypeInt8:
            {
                dataSize = sizeof(int8_t);
                address = ((int8_t*)data) + startIndex;
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
                address = ((uint32_t*)data) + startIndex;
                break;
            }
            case PortDataTypeUInt16:
            {
                dataSize = sizeof(uint16_t);
                address = ((uint16_t*)data) + startIndex;
                break;
            }
            case PortDataTypeUInt8:
            {
                dataSize = sizeof(uint8_t);
                address = ((uint8_t*)data) + startIndex;
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
        const void * address = ((bool*)data) + startIndex;

        memcpy(contiguousData, address, dataSize * length);
    }


}
