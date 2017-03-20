#ifndef WBT_SIMULATORSYNCHRONIZER_H
#define WBT_SIMULATORSYNCHRONIZER_H

#include "Block.h"

namespace wbt {
    class SimulatorSynchronizer;
}

class wbt::SimulatorSynchronizer : public wbt::Block {
public:
    static std::string ClassName;

    SimulatorSynchronizer();
    virtual ~SimulatorSynchronizer();

    virtual unsigned numberOfParameters();
    virtual std::vector<std::string> additionalBlockOptions();
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);

private:

    double m_period;
    bool m_firstRun;

    struct RPCData;
    RPCData *m_rpcData;
};

#endif /* end of include guard: WBT_SIMULATORSYNCHRONIZER_H */
