#include "Block.h"
#include <memory>

#ifndef WBT_MINJERKTRAJGENERATOR_H
#define WBT_MINJERKTRAJGENERATOR_H

namespace wbt {
    class MinimumJerkTrajectoryGenerator;
    class BlockInformation;
} // namespace wbt

namespace iCub {
    namespace ctrl {
        class minJerkTrajGen;
    }
} // namespace iCub

namespace yarp {
    namespace sig {
        class Vector;
    }
} // namespace yarp

class wbt::MinimumJerkTrajectoryGenerator : public wbt::Block
{
public:
    static const std::string ClassName;

    MinimumJerkTrajectoryGenerator();
    ~MinimumJerkTrajectoryGenerator() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
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
};

#endif /* end of include guard: WBT_MINJERKTRAJGENERATOR_H */
