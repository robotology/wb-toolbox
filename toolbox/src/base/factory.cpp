#include "toolbox.h"

wbt::Block* wbt::Block::instantiateBlockWithClassName(std::string blockClassName)
{
    wbt::Block *block = NULL;

    if (blockClassName == wbt::YarpRead::ClassName) {
        block = new wbt::YarpRead();
    } else if (blockClassName == wbt::YarpWrite::ClassName) {
        block = new wbt::YarpWrite();
    } else if (blockClassName == wbt::YARPWBIConverter::ClassName) {
        block = new wbt::YARPWBIConverter();
    } else if (blockClassName == wbt::YarpClock::ClassName) {
        block = new wbt::YarpClock();
    } else if (blockClassName == wbt::MassMatrix::ClassName) {
        block = new wbt::MassMatrix();
    } else if (blockClassName == wbt::ForwardKinematics::ClassName) {
        block = new wbt::ForwardKinematics();
    } else if (blockClassName == wbt::SetReferences::ClassName) {
        block = new wbt::SetReferences();
    } else if (blockClassName == wbt::RealTimeSynchronizer::ClassName) {
        block = new wbt::RealTimeSynchronizer();
    } else if (blockClassName == wbt::SimulatorSynchronizer::ClassName) {
        block = new wbt::SimulatorSynchronizer();
    } else if (blockClassName == wbt::Jacobian::ClassName) {
        block = new wbt::Jacobian();
    } else if (blockClassName == wbt::GetEstimate::ClassName) {
        block = new wbt::GetEstimate();
    } else if (blockClassName == wbt::InverseDynamics::ClassName) {
        block = new wbt::InverseDynamics();
    } else if (blockClassName == wbt::DotJNu::ClassName) {
        block = new wbt::DotJNu();
    } else if (blockClassName == wbt::GetLimits::ClassName) {
        block = new wbt::GetLimits();
    } else if (blockClassName == wbt::CentroidalMomentum::ClassName) {
        block = new wbt::CentroidalMomentum();
    }
#ifdef WBT_USES_ICUB
    else if (blockClassName == wbt::MinimumJerkTrajectoryGenerator::ClassName) {
        block = new wbt::MinimumJerkTrajectoryGenerator();
    } 
#endif
#ifdef WBT_USES_IPOPT
    else if (blockClassName == wbt::InverseKinematics::ClassName) {
        block = new wbt::InverseKinematics();
    }
#endif
    else if (blockClassName == wbt::RemoteInverseKinematics::ClassName) {
        block = new wbt::RemoteInverseKinematics();
    }
#ifdef WBT_USES_CODYCO_COMMONS
    else if (blockClassName == wbt::SetLowLevelPID::ClassName) {
        block = new wbt::SetLowLevelPID();
    }
#endif

    
    return block;
}
