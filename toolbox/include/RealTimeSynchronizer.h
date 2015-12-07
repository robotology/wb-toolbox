#ifndef WBT_REALTIMESYNCHRONIZER_H
#define WBT_REALTIMESYNCHRONIZER_H

#include "Block.h"

namespace wbt {
    class RealTimeSynchronizer;
}

class wbt::RealTimeSynchronizer : public wbt::Block {
public:
    static std::string ClassName;
    
    RealTimeSynchronizer();
    virtual ~RealTimeSynchronizer();
    
    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);
    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
private:
    double m_period;

    double m_initialTime;
    unsigned long m_counter;
};

#endif /* end of include guard: WBT_REALTIMESYNCHRONIZER_H */
