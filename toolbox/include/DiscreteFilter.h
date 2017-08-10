#include "Block.h"
#include <string>
#include <vector>

#ifndef WBT_FILTER_H
#define WBT_FILTER_H

namespace wbt {
    class DiscreteFilter;
} // namespace wbt

namespace iCub {
    namespace ctrl {
        class IFilter;
    }
} // namespace iCub

namespace yarp {
    namespace sig {
        class Vector;
    }
} // namespace yarp

class wbt::DiscreteFilter : public wbt::Block {
private:
    bool firstRun;
    iCub::ctrl::IFilter* filter;
    yarp::sig::Vector* num;
    yarp::sig::Vector* den;
    yarp::sig::Vector* inputSignalVector;

    static void stringToYarpVector(const std::string s, yarp::sig::Vector* v);

public:
    static std::string ClassName;

    DiscreteFilter();
    ~DiscreteFilter() = default;

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(BlockInformation* blockInfo, wbt::Error* error);
    virtual bool initialize(BlockInformation* blockInfo, wbt::Error* error);
    virtual bool terminate(BlockInformation* blockInfo, wbt::Error* error);
    virtual bool output(BlockInformation* blockInfo, wbt::Error* error);
};

#endif // WBT_FILTER_H
