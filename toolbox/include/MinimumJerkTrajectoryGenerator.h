#include "Block.h"

#ifndef WBT_MINJERKTRAJGENERATOR_H
#define WBT_MINJERKTRAJGENERATOR_H

namespace wbt {
    class MinimumJerkTrajectoryGenerator;
    class BlockInformation;
}

namespace iCub {
    namespace ctrl {
        class minJerkTrajGen;
    }
}

namespace yarp {
    namespace sig {
        class Vector;
    }
}

class wbt::MinimumJerkTrajectoryGenerator : public wbt::Block {
public:
    static std::string ClassName;
    
    MinimumJerkTrajectoryGenerator();
    
    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);
    
private:

    iCub::ctrl::minJerkTrajGen *m_generator;

    int m_outputFirstDerivativeIndex;
    int m_outputSecondDerivativeIndex;

    double m_previousSettlingTime;

    bool m_firstRun;
    bool m_explicitInitialValue;
    bool m_externalSettlingTime;
    bool m_resetOnSettlingTimeChange;
    yarp::sig::Vector *m_initialValues;
    yarp::sig::Vector *m_reference;
    
};

#endif /* end of include guard: WBT_MINJERKTRAJGENERATOR_H */
