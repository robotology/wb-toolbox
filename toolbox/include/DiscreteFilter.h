#include "Block.h"
#include <memory>
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

class wbt::DiscreteFilter : public wbt::Block
{
private:
    unsigned inputSignalWidth;
    std::unique_ptr<iCub::ctrl::IFilter> filter;
    std::unique_ptr<yarp::sig::Vector> y0;
    std::unique_ptr<yarp::sig::Vector> u0;
    std::unique_ptr<yarp::sig::Vector> inputSignalVector;

    static void stringToYarpVector(const std::string s, yarp::sig::Vector& v);


public:
    static const std::string ClassName;

    DiscreteFilter();
    ~DiscreteFilter() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool initializeInitialConditions(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_FILTER_H
