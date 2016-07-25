#include "Block.h"

#ifndef WBT_MINJERKTRAJGENERATOR_H
#define WBT_MINJERKTRAJGENERATOR_H

namespace wbt {
    class MinimumJerkTrajectoryGenerator;
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
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);
    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
private:

    iCub::ctrl::minJerkTrajGen *m_generator;

    int_T m_outputFirstDerivativeIndex;
    int_T m_outputSecondDerivativeIndex;

    double m_previousSettlingTime;

    bool m_firstRun;
    bool m_explicitInitialValue;
    bool m_externalSettlingTime;
    bool m_resetOnSettlingTimeChange;
    yarp::sig::Vector *m_initialValues;
    yarp::sig::Vector *m_reference;
    
};

#endif /* end of include guard: WBT_MINJERKTRAJGENERATOR_H */
