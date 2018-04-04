#include "Signal.h"

using namespace wbt;

const int Signal::DynamicSize = -1;

Signal::~Signal()
{
    deleteBuffer();
}

Signal::Signal(const Signal& signal)
    : m_width(signal.m_width)
    , m_isConst(signal.m_isConst)
    , m_portDataType(signal.m_portDataType)
    , m_dataFormat(signal.m_dataFormat)
    , m_bufferPtr(nullptr)
{
    if (signal.m_bufferPtr) {
        switch (signal.m_dataFormat) {
            // Just copy the pointer to MATLAB's memory
            case DataFormat::CONTIGUOUS_ZEROCOPY:
                m_bufferPtr = signal.m_bufferPtr;
                break;
            // Copy the allocated data
            case DataFormat::NONCONTIGUOUS:
            case DataFormat::CONTIGUOUS:
                allocateBuffer(signal.m_bufferPtr, m_bufferPtr, signal.m_width);
                break;
        }
    }
}

Signal::Signal(const DataFormat& dataFormat, const DataType& dataType, const bool& isConst)
    : m_isConst(isConst)
    , m_portDataType(dataType)
    , m_dataFormat(dataFormat)
    , m_bufferPtr(nullptr)
{}

Signal::Signal(Signal&& other)
    : m_width(other.m_width)
    , m_isConst(other.m_isConst)
    , m_portDataType(other.m_portDataType)
    , m_dataFormat(other.m_dataFormat)
    , m_bufferPtr(other.m_bufferPtr)
{
    other.m_width = 0;
    other.m_bufferPtr = nullptr;
}

void Signal::allocateBuffer(const void* const bufferInput, void*& bufferOutput, unsigned length)
{
    switch (m_portDataType) {
        case DataType::DOUBLE: {
            // Allocate the array
            bufferOutput = static_cast<void*>(new double[m_width]);
            // Cast to double
            const double* const bufferInputDouble = static_cast<const double*>(bufferInput);
            double* bufferOutputDouble = static_cast<double*>(bufferOutput);
            // Copy data
            std::copy(bufferInputDouble, bufferInputDouble + length, bufferOutputDouble);
            return;
        }
        default:
            // TODO: Implement other DataType
            wbtError << "The specified DataType is not yet supported. Used DOUBLE instead.";
            return;
    }
}

void Signal::deleteBuffer()
{
    if (m_dataFormat == DataFormat::CONTIGUOUS_ZEROCOPY || !m_bufferPtr) {
        return;
    }

    switch (m_portDataType) {
        case DataType::DOUBLE:
            delete static_cast<double*>(m_bufferPtr);
            m_bufferPtr = nullptr;
            return;
        default:
            // TODO: Implement other DataType
            wbtError << "The specified DataType is not yet supported. Used DOUBLE instead.";
            return;
    }
}

bool Signal::initializeBufferFromContiguousZeroCopy(const void* buffer)
{
    if (m_dataFormat != DataFormat::CONTIGUOUS_ZEROCOPY) {
        wbtError << "Trying to initialize a CONTIGUOUS_ZEROCOPY signal but the configured "
                 << "DataFormat does not match.";
        return false;
    }

    m_bufferPtr = const_cast<void*>(buffer);
    return true;
}

bool Signal::initializeBufferFromContiguous(const void* buffer)
{
    if (m_dataFormat != DataFormat::CONTIGUOUS || m_width <= 0) {
        wbtError << "Trying to initialize a CONTIGUOUS signal but the configured "
                 << "DataFormat does not match.";
        return false;
    }

    if (m_portDataType == DataType::DOUBLE) {
        // Allocate a new vector to store data from the non-contiguous signal
        m_bufferPtr = static_cast<void*>(new double[m_width]);
        const double* bufferInputDouble = static_cast<const double*>(buffer);
        double* bufferOutputDouble = static_cast<double*>(m_bufferPtr);

        // Copy data from the input memory location
        std::copy(bufferInputDouble, bufferInputDouble + m_width, bufferOutputDouble);
    }
    return true;
}

bool Signal::initializeBufferFromNonContiguous(const void* const* bufferPtrs)
{
    if (m_dataFormat != DataFormat::NONCONTIGUOUS || m_width <= 0) {
        wbtError << "Trying to initialize a NONCONTIGUOUS signal but the configured "
                 << "DataFormat does not match.";
        return false;
    }

    if (m_portDataType == DataType::DOUBLE) {
        // Allocate a new vector to store data from the non-contiguous signal
        m_bufferPtr = static_cast<void*>(new double[m_width]);
        double* bufferPtrDouble = static_cast<double*>(m_bufferPtr);

        // Copy data from MATLAB's memory to the Signal object
        for (auto i = 0; i < m_width; ++i) {
            const double* valuePtr = static_cast<const double*>(*bufferPtrs);
            bufferPtrDouble[i] = valuePtr[i];
        }
    }
    return true;
}

void Signal::setWidth(const unsigned& width)
{
    m_width = width;
}

unsigned Signal::getWidth() const
{
    return m_width;
}

DataType Signal::getPortDataType() const
{
    return m_portDataType;
}

bool Signal::isConst() const
{
    return m_isConst;
}

Signal::DataFormat Signal::getDataFormat() const
{
    return m_dataFormat;
}

bool Signal::set(const unsigned& index, const double& data)
{
    if (m_isConst || m_width <= index) {
        wbtError << "The signal is either const or the index exceeds its width.";
        return false;
    }

    if (!m_bufferPtr) {
        wbtError << "The pointer to data is null. The signal was not configured properly.";
        return false;
    }

    switch (m_portDataType) {
        case DataType::DOUBLE: {
            double* buffer = static_cast<double*>(m_bufferPtr);
            buffer[index] = data;
            break;
        }
        case DataType::SINGLE: {
            float* buffer = static_cast<float*>(m_bufferPtr);
            buffer[index] = data;
            break;
        }
        default:
            // TODO: Implement other DataType
            wbtError << "The specified DataType is not yet supported. Used DOUBLE instead.";
            return false;
            break;
    }
    return true;
}

// Explicit template instantiations
// ================================
template double* Signal::getBuffer<double>() const;
template double Signal::get<double>(const unsigned& i) const;
template bool Signal::setBuffer<double>(const double* data, const unsigned& length);

// Template definitions
// ===================

template <typename T>
T Signal::get(const unsigned& i) const
{
    T* buffer = getBuffer<T>();

    if (!buffer) {
        return false;
    }

    if (i >= m_width) {
        wbtError << "Trying to access an element that exceeds signal width.";
        return false;
    }

    return buffer[i];
}

template <typename T>
T* Signal::getBuffer() const
{
    const std::map<DataType, size_t> mapDataTypeToHash = {
        {DataType::DOUBLE, typeid(double).hash_code()},
        {DataType::SINGLE, typeid(float).hash_code()},
        {DataType::INT8, typeid(int8_t).hash_code()},
        {DataType::UINT8, typeid(uint8_t).hash_code()},
        {DataType::INT16, typeid(int16_t).hash_code()},
        {DataType::UINT16, typeid(uint16_t).hash_code()},
        {DataType::INT32, typeid(int32_t).hash_code()},
        {DataType::UINT32, typeid(uint32_t).hash_code()},
        {DataType::BOOLEAN, typeid(bool).hash_code()}};

    if (!m_bufferPtr) {
        wbtError << "The pointer to data is null. The signal was not configured properly.";
        return nullptr;
    }

    // Check the returned matches the same type of the portType.
    // If this is not met, applying pointer arithmetics on the returned
    // pointer would show unknown behaviour.
    if (typeid(T).hash_code() != mapDataTypeToHash.at(m_portDataType)) {
        wbtError << "Trying to get the buffer using a type different that its DataType";
        return nullptr;
    }

    // Return the correct pointer
    return static_cast<T*>(m_bufferPtr);
}

template <typename T>
bool Signal::setBuffer(const T* data, const unsigned& length)
{
    // Non contiguous signals follow the Simulink convention of being read-only.
    // They are used only for input signals.
    if (m_dataFormat == DataFormat::NONCONTIGUOUS || m_isConst) {
        wbtError << "Changing buffer address to NONCONTIGUOUS and const signals is not allowed.";
        return false;
    }

    // Fail if the length is greater of the signal width
    if (m_dataFormat == DataFormat::CONTIGUOUS_ZEROCOPY && length > m_width) {
        wbtError << "Trying to set a buffer with a length greater than the signal width.";
        return false;
    }

    // Check that T matches the type of raw buffer stored. Use getBuffer since it will return
    // nullptr if this is not met.
    if (!getBuffer<T>()) {
        wbtError << "Trying to get a pointer with a type not matching the signal's DataType.";
        return false;
    }

    switch (m_dataFormat) {
        case DataFormat::CONTIGUOUS:
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
        case DataFormat::CONTIGUOUS_ZEROCOPY:
            // Reset current data
            std::fill(getBuffer<T>(), getBuffer<T>() + m_width, 0);
            // Copy new data
            std::copy(data, data + length, getBuffer<T>());
            // Update the width
            m_width = length;
            break;
        case DataFormat::NONCONTIGUOUS:
            wbtError << "The code should never arrive here. Unexpected error.";
            return false;
    }

    return true;
}
