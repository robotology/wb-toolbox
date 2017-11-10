#ifndef WBT_REALTIMESYNCHRONIZER_H
#define WBT_REALTIMESYNCHRONIZER_H

#include "Block.h"

namespace wbt {
    class RealTimeSynchronizer;
}

class wbt::RealTimeSynchronizer : public wbt::Block
{
public:
    static const std::string ClassName;

    RealTimeSynchronizer();
    ~RealTimeSynchronizer() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;

private:
    double m_period;

    double m_initialTime;
    unsigned long m_counter;

    static const unsigned PARAM_PERIOD; // Period
};

#endif /* end of include guard: WBT_REALTIMESYNCHRONIZER_H */
