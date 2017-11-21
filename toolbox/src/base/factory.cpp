#include "toolbox.h"

using namespace wbt;

Block* Block::instantiateBlockWithClassName(std::string blockClassName)
{
    Block* block = nullptr;

    if (blockClassName == YarpRead::ClassName) {
        block = new YarpRead();
    } else if (blockClassName == YarpWrite::ClassName) {
        block = new YarpWrite();
    } else if (blockClassName == ModelPartitioner::ClassName) {
        block = new ModelPartitioner();
    } else if (blockClassName == YarpClock::ClassName) {
        block = new YarpClock();
    } else if (blockClassName == MassMatrix::ClassName) {
        block = new MassMatrix();
    } else if (blockClassName == ForwardKinematics::ClassName) {
        block = new ForwardKinematics();
    } else if (blockClassName == SetReferences::ClassName) {
        block = new SetReferences();
    } else if (blockClassName == RealTimeSynchronizer::ClassName) {
        block = new RealTimeSynchronizer();
    } else if (blockClassName == SimulatorSynchronizer::ClassName) {
        block = new SimulatorSynchronizer();
    } else if (blockClassName == Jacobian::ClassName) {
        block = new Jacobian();
    } else if (blockClassName == GetMeasurement::ClassName) {
        block = new GetMeasurement();
    } else if (blockClassName == InverseDynamics::ClassName) {
        block = new InverseDynamics();
    } else if (blockClassName == DotJNu::ClassName) {
        block = new DotJNu();
    } else if (blockClassName == GetLimits::ClassName) {
        block = new GetLimits();
    } else if (blockClassName == CentroidalMomentum::ClassName) {
        block = new CentroidalMomentum();
    } else if (blockClassName == SetLowLevelPID::ClassName) {
        block = new SetLowLevelPID();
    }
#ifdef WBT_USES_ICUB
    else if (blockClassName == MinimumJerkTrajectoryGenerator::ClassName) {
        block = new MinimumJerkTrajectoryGenerator();
    }
#endif
#ifdef WBT_USES_IPOPT
    // else if (blockClassName == InverseKinematics::ClassName) {
    //     block = new InverseKinematics();
    // }
#endif
    // else if (blockClassName == RemoteInverseKinematics::ClassName) {
    //     block = new RemoteInverseKinematics();
    // }
    return block;
}
