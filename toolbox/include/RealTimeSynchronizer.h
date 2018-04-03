#ifndef WBT_REALTIMESYNCHRONIZER_H
#define WBT_REALTIMESYNCHRONIZER_H

#include "Block.h"

namespace wbt {
    class RealTimeSynchronizer;
}

class wbt::RealTimeSynchronizer : public wbt::Block
{
private:
    double m_period = 0.01;
    double m_initialTime;
    unsigned long m_counter;

public:
    static const std::string ClassName;

    RealTimeSynchronizer() = default;
    ~RealTimeSynchronizer() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* end of include guard: WBT_REALTIMESYNCHRONIZER_H */
