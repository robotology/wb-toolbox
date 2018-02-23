#ifndef WBT_SIGNAL_H
#define WBT_SIGNAL_H

#include "BlockInformation.h"

#include <cassert>
#include <memory>

namespace wbt {
    class Signal;
    enum SignalDataFormat
    {
        NONCONTIGUOUS = 0,
        CONTIGUOUS = 1,
        CONTIGUOUS_ZEROCOPY = 2
    };
} // namespace wbt

class wbt::Signal
{
private:
    int m_width;
    const bool m_isConst;
    const PortDataType m_portDataType;
    const SignalDataFormat m_dataFormat;

    void* m_bufferPtr;

    void deleteBuffer();
    void allocateBuffer(const void* const bufferInput, void*& bufferOutput, unsigned length);

    template <typename T>
    T* getCastBuffer() const;

public:
    // Ctor and Dtor
    Signal(const SignalDataFormat& dataFormat = CONTIGUOUS_ZEROCOPY,
           const PortDataType& dataType = PortDataTypeDouble,
           const bool& isConst = true);
    ~Signal();
    // Copy
    Signal(const Signal& signal);
    Signal& operator=(const Signal& signal) = delete;
    // Move
    Signal(Signal&& signal);
    Signal& operator=(Signal&& signal) = delete;

    bool initializeBufferFromContiguous(const void* buffer);
    bool initializeBufferFromContiguousZeroCopy(const void* buffer);
    bool initializeBufferFromNonContiguous(const void* const* bufferPtrs);

    bool isConst() const;
    unsigned getWidth() const;
    PortDataType getPortDataType() const;
    SignalDataFormat getDataFormat() const;
    template <typename T>
    T* getBuffer() const;
    template <typename T>
    T get(const unsigned& i) const;

    void setWidth(const unsigned& width);
    bool set(const unsigned& index, const double& data);
    template <typename T>
    bool setBuffer(const T* data, const unsigned& length);
};

template <typename T>
T* wbt::Signal::getBuffer() const
{
    // Check the returned matches the same type of the portType.
    // If this is not met, appliying pointer arithmetics on the returned
    // pointer would show unknown behaviour.
    switch (m_portDataType) {
        case wbt::PortDataTypeDouble:
            if (typeid(T).hash_code() != typeid(double).hash_code()) {
                return nullptr;
            }
            break;
        case wbt::PortDataTypeSingle:
            if (typeid(T).hash_code() != typeid(float).hash_code()) {
                return nullptr;
            }
            break;
        case wbt::PortDataTypeInt8:
            if (typeid(T).hash_code() != typeid(int8_t).hash_code()) {
                return nullptr;
            }
            break;
        case wbt::PortDataTypeUInt8:
            if (typeid(T).hash_code() != typeid(uint8_t).hash_code()) {
                return nullptr;
            }
            break;
        case wbt::PortDataTypeInt16:
            if (typeid(T).hash_code() != typeid(int16_t).hash_code()) {
                return nullptr;
            }
            break;
        case wbt::PortDataTypeUInt16:
            if (typeid(T).hash_code() != typeid(uint16_t).hash_code()) {
                return nullptr;
            }
            break;
        case wbt::PortDataTypeInt32:
            if (typeid(T).hash_code() != typeid(int32_t).hash_code()) {
                return nullptr;
            }
            break;
        case wbt::PortDataTypeUInt32:
            if (typeid(T).hash_code() != typeid(uint32_t).hash_code()) {
                return nullptr;
            }
            break;
        case wbt::PortDataTypeBoolean:
            if (typeid(T).hash_code() != typeid(bool).hash_code()) {
                return nullptr;
            }
            break;
        default:
            return nullptr;
            break;
    }

    // Return the correct pointer
    return static_cast<T*>(m_bufferPtr);
}

template <typename T>
bool wbt::Signal::setBuffer(const T* data, const unsigned& length)
{
    // Non contiguous signals follow the Simulink convention of being read-only
    if (m_dataFormat == NONCONTIGUOUS || m_isConst) {
        return false;
    }

    if (m_dataFormat == CONTIGUOUS_ZEROCOPY && length > m_width) {
        return false;
    }

    if (typeid(getBuffer<T>()).hash_code() != typeid(T*).hash_code()) {
        return false;
    }

    switch (m_dataFormat) {
        case CONTIGUOUS:
            // Delete the current array
            if (m_bufferPtr) {
                delete getBuffer<T>();
                m_bufferPtr = nullptr;
                m_width = 0;
            }
            // Allocate a new empty array
            m_bufferPtr = static_cast<void*>(new T[length]);
            m_width = length;
            // Fill it with new data
            std::copy(data, data + length, getBuffer<T>());
            break;
        case CONTIGUOUS_ZEROCOPY:
            // Reset current data
            std::fill(getBuffer<T>(), getBuffer<T>() + m_width, 0);
            // Copy new data
            std::copy(data, data + length, getBuffer<T>());
            // Update the width
            m_width = length;
            break;
        case NONCONTIGUOUS:
            return false;
    }

    return true;
}

template <typename T>
T wbt::Signal::get(const unsigned& i) const
{
    T* buffer = getBuffer<T>();
    assert(buffer);

    return buffer[i];
}

#endif /* end of include guard: WBT_SIGNAL_H */
