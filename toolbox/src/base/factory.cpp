#include "toolbox.h"

wbt::Block* wbt::Block::instantiateBlockWithClassName(std::string blockClassName)
{
    wbt::Block *block = NULL;
    
    if (blockClassName == wbt::YarpRead::ClassName) {
        block = new wbt::YarpRead();
    } else if (blockClassName == wbt::YarpWrite::ClassName) {
        block = new wbt::YarpWrite();
    } else if (blockClassName == wbt::MassMatrix::ClassName) {
        block = new wbt::MassMatrix();
    } else if (blockClassName == wbt::GetBiasForces::ClassName) {
        block = new wbt::GetBiasForces();
    } else if (blockClassName == wbt::ForwardKinematics::ClassName) {
        block = new wbt::ForwardKinematics();
    } else if (blockClassName == wbt::SetReferences::ClassName) {
        block = new wbt::SetReferences();
    } else if (blockClassName == wbt::RealTimeSynchronizer::ClassName) {
        block = new wbt::RealTimeSynchronizer();
    } else if (blockClassName == wbt::SimulatorSynchronizer::ClassName) {
        block = new wbt::SimulatorSynchronizer();
    }
    
    return block;
}
