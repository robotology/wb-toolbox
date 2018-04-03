#include "Signal.h"

using namespace wbt;

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
            case CONTIGUOUS_ZEROCOPY:
                m_bufferPtr = signal.m_bufferPtr;
                break;
            // Copy the allocated data
            case NONCONTIGUOUS:
            case CONTIGUOUS:
                allocateBuffer(signal.m_bufferPtr, m_bufferPtr, signal.m_width);
                break;
        }
    }
}

Signal::Signal(const SignalDataFormat& dataFormat,
               const PortDataType& dataType,
               const bool& isConst)
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
    // TODO: Implement other PortDataType
    switch (m_portDataType) {
        case PortDataTypeDouble: {
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
    if (m_dataFormat == CONTIGUOUS_ZEROCOPY || !m_bufferPtr) {
        return;
    }

    // TODO: Implement other PortDataType
    switch (m_portDataType) {
        case PortDataTypeDouble:
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
    if (m_dataFormat != CONTIGUOUS_ZEROCOPY) {
        wbtError << "Trying to initialize a CONTIGUOUS_ZEROCOPY signal but the configured "
                 << "DataFormat does not match.";
        return false;
    }

    m_bufferPtr = const_cast<void*>(buffer);
    return true;
}

bool Signal::initializeBufferFromContiguous(const void* buffer)
{
    if (m_dataFormat != CONTIGUOUS || m_width <= 0) {
        wbtError << "Trying to initialize a CONTIGUOUS signal but the configured "
                 << "DataFormat does not match.";
        return false;
    }

    if (m_portDataType == PortDataTypeDouble) {
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
    if (m_dataFormat != NONCONTIGUOUS || m_width <= 0) {
        wbtError << "Trying to initialize a NONCONTIGUOUS signal but the configured "
                 << "DataFormat does not match.";
        return false;
    }

    if (m_portDataType == PortDataTypeDouble) {
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

PortDataType Signal::getPortDataType() const
{
    return m_portDataType;
}

bool Signal::isConst() const
{
    return m_isConst;
}

SignalDataFormat Signal::getDataFormat() const
{
    return m_dataFormat;
}

bool Signal::set(const unsigned& index, const double& data)
{
    if (m_isConst || m_width <= index) {
        wbtError << "The signal is either const or the index exceeds its width.";
        return false;
    }

    // TODO: Implement other PortDataType
    if (!m_bufferPtr) {
        wbtError << "The pointer to data is null. The signal was not configured properly.";
        return false;
    }

    switch (m_portDataType) {
        case PortDataTypeDouble: {
            double* buffer = static_cast<double*>(m_bufferPtr);
            buffer[index] = data;
            break;
        }
        case PortDataTypeSingle: {
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
