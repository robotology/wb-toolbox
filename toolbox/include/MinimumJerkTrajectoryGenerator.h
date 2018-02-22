#include "Block.h"
#include <memory>

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
    static const std::string ClassName;

    MinimumJerkTrajectoryGenerator();
    ~MinimumJerkTrajectoryGenerator() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;

private:

    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_generator;

    int m_outputFirstDerivativeIndex;
    int m_outputSecondDerivativeIndex;

    double m_previousSettlingTime;

    bool m_firstRun;
    bool m_explicitInitialValue;
    bool m_externalSettlingTime;
    bool m_resetOnSettlingTimeChange;
    std::unique_ptr<yarp::sig::Vector> m_initialValues;
    std::unique_ptr<yarp::sig::Vector> m_reference;

    static const unsigned PARAM_IDX_SAMPLE_TIME;           // Sample Time (double)
    static const unsigned PARAM_IDX_SETTLING_TIME;         // Settling Time (double)
    static const unsigned PARAM_IDX_OUTPUT_1ST_DERIVATIVE; // Output first derivative (boolean)
    static const unsigned PARAM_IDX_OUTPUT_2ND_DERIVATIVE; // Output second derivative (boolean)
    static const unsigned PARAM_IDX_INITIAL_VALUE;         // Initial signal value as input (boolean)
    static const unsigned PARAM_IDX_EXT_SETTLINGTIME;      // Control if the settling time comes from
                                                           // external port or static parameter
    static const unsigned PARAM_IDX_RESET_CHANGEST;        // True if the block should reset the traj
                                                           // generator in case settling time changes
};

#endif /* end of include guard: WBT_MINJERKTRAJGENERATOR_H */
