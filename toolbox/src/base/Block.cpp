#include "Block.h"
#include "toolbox.h"

wbit::Block::~Block() {}
void wbit::Block::parameterAtIndexIsTunable(unsigned index, bool &tunable) { tunable = false; }
bool wbit::Block::checkParameters(SimStruct *S, wbit::Error *error) { return true; }


wbit::Block* wbit::Block::instantiateBlockWithClassName(std::string blockClassName)
{
    wbit::Block *block = NULL;
    
    if (blockClassName == wbit::YarpRead::ClassName) {
        block = new wbit::YarpRead();
    } else if (blockClassName == wbit::MassMatrix::ClassName) {
        block = new wbit::MassMatrix();
    } else if (blockClassName == wbit::GetBiasForces::ClassName) {
        block = new wbit::GetBiasForces();
    } else if (blockClassName == wbit::ForwardKinematics::ClassName) {
        block = new wbit::ForwardKinematics();
    }
    
    return block;
}
