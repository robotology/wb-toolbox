/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_SIGNAL_H
#define WBT_SIGNAL_H

#include "BlockInformation.h"

#include <cassert>
#include <map>
#include <memory>

namespace wbt {
    class Signal;
    enum class DataType;
} // namespace wbt

enum class wbt::DataType
{
    DOUBLE,
    SINGLE,
    INT8,
    UINT8,
    INT16,
    UINT16,
    INT32,
    UINT32,
    BOOLEAN,
};

class wbt::Signal
{
public:
    enum class DataFormat
    {
        NONCONTIGUOUS = 0,
        CONTIGUOUS = 1,
        CONTIGUOUS_ZEROCOPY = 2
    };

private:
    int m_width = DynamicSize;
    const bool m_isConst;
    const DataType m_portDataType;
    const DataFormat m_dataFormat;

    void* m_bufferPtr;

    void deleteBuffer();
    void allocateBuffer(const void* const bufferInput, void*& bufferOutput, unsigned length);

public:
    static const int DynamicSize;

    // Ctor and Dtor
    Signal(const DataFormat& dataFormat = DataFormat::CONTIGUOUS_ZEROCOPY,
           const DataType& dataType = DataType::DOUBLE,
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

    bool isValid() const;

    bool isConst() const;
    int getWidth() const;
    DataType getPortDataType() const;
    DataFormat getDataFormat() const;

    template <typename T>
    T* getBuffer() const;

    template <typename T>
    T get(const unsigned& i) const;

    void setWidth(const unsigned& width);
    bool set(const unsigned& index, const double& data);
    template <typename T>
    bool setBuffer(const T* data, const unsigned& length);
};

// Explicit declaration of templates for all the supported types
// =============================================================

// TODO: for the time being, only DOUBLE is allowed. The toolbox has an almost complete support to
//       many other data types, but they need to be tested.

namespace wbt {
    // DataType::DOUBLE
    extern template double* Signal::getBuffer<double>() const;
    extern template double Signal::get<double>(const unsigned& i) const;
    extern template bool Signal::setBuffer<double>(const double* data, const unsigned& length);
} // namespace wbt

#endif // WBT_SIGNAL_H
