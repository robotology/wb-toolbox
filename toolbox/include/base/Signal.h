#ifndef WBT_SIGNAL_H
#define WBT_SIGNAL_H

#include "BlockInformation.h"
#include <vector>

namespace wbt {
    class Signal;
}

class wbt::Signal
{
private:
    PortDataType portType;
    bool isContiguous;
    bool isConstPort;

    void** nonContiguousData;
    void* contiguousData;

public:
    Signal();

    void initSignalType(wbt::PortDataType type, bool constPort);

    void setContiguousBuffer(void* buffer);
    void setContiguousBuffer(const void* buffer);
    void setNonContiguousBuffer(void** buffer);
    void setNonContiguousBuffer(const void* const* buffer);

    const Data get(unsigned index) const;
    void* getContiguousBuffer();
    std::vector<double> getStdVector(unsigned length) const;

    //the missing are cast
    void set(unsigned index, double data);
    void setBuffer(const double* data, const unsigned length, unsigned startIndex = 0);

    void set(unsigned index, int32_t data);
    void setBuffer(const int32_t* data, const unsigned length, unsigned startIndex = 0);

    void set(unsigned index, uint32_t data);
    void setBuffer(const uint32_t* data, const unsigned length, unsigned startIndex = 0);

    void set(unsigned index, bool data);
    void setBuffer(const bool* data, const unsigned length, unsigned startIndex = 0);

};


#endif /* end of include guard: WBT_SIGNAL_H */
