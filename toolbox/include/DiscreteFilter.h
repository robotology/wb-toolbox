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

    // Parameters
    static const unsigned PARAM_IDX_FLT_TYPE;
    static const unsigned PARAM_IDX_NUMCOEFF;
    static const unsigned PARAM_IDX_DENCOEFF;
    static const unsigned PARAM_IDX_1LOWP_FC;
    static const unsigned PARAM_IDX_1LOWP_TS;
    static const unsigned PARAM_IDX_MD_ORDER;
    static const unsigned PARAM_IDX_INIT_Y0;
    static const unsigned PARAM_IDX_INIT_U0;
    // Inputs
    static const unsigned INPUT_IDX_SIGNAL;
    // Outputs
    static const unsigned OUTPUT_IDX_SIGNAL;
    // Other defines
    static const int SIGNAL_DYNAMIC_SIZE;

public:
    static const std::string ClassName;

    DiscreteFilter();
    ~DiscreteFilter() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(const BlockInformation* blockInfo) override;
    bool initializeInitialConditions(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_FILTER_H
