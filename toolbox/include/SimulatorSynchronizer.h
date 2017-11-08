#ifndef WBT_SIMULATORSYNCHRONIZER_H
#define WBT_SIMULATORSYNCHRONIZER_H

#include "Block.h"

namespace wbt {
    class SimulatorSynchronizer;
}

class wbt::SimulatorSynchronizer : public wbt::Block
{
public:
    static const std::string ClassName;

    SimulatorSynchronizer();
    ~SimulatorSynchronizer() override = default;

    unsigned numberOfParameters() override;
    std::vector<std::string> additionalBlockOptions() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(BlockInformation* blockInfo) override;
    bool output(BlockInformation* blockInfo) override;

private:
    double m_period;
    bool m_firstRun;

    struct RPCData;
    RPCData* m_rpcData;

    static const unsigned PARAM_PERIOD;     // Period
    static const unsigned PARAM_GZCLK_PORT; // Gazebo clock port
    static const unsigned PARAM_RPC_PORT;   // RPC client port name
};

#endif /* end of include guard: WBT_SIMULATORSYNCHRONIZER_H */
