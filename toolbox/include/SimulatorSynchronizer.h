#include "Block.h"

#ifndef WBT_SIMULATORSYNCHRONIZER_H
#define WBT_SIMULATORSYNCHRONIZER_H

namespace wbt {
    class SimulatorSynchronizer;
}

class wbt::SimulatorSynchronizer : public wbt::Block {
public:
    static std::string ClassName;
    
    SimulatorSynchronizer();
    virtual ~SimulatorSynchronizer();
    
    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);
    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
private:

    double m_period;
    bool m_firstRun;

    struct RPCData;
    RPCData *m_rpcData;
};

#endif /* end of include guard: WBT_SIMULATORSYNCHRONIZER_H */
