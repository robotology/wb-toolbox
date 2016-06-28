#include "Block.h"
#include "toolbox.h"

wbt::Block::~Block() {}
std::vector<std::string> wbt::Block::additionalBlockOptions() { return std::vector<std::string>(); }
void wbt::Block::parameterAtIndexIsTunable(unsigned /*index*/, bool &tunable) { tunable = false; }
bool wbt::Block::checkParameters(wbt::BlockInformation */*blockInfo*/, wbt::Error */*error*/) { return true; }

unsigned wbt::Block::numberOfDiscreteStates() { return 0; }
unsigned wbt::Block::numberOfContinuousStates() { return 0; }

bool wbt::Block::updateDiscreteState(wbt::BlockInformation */*blockInfo*/, wbt::Error */*error*/) { return true; }
bool wbt::Block::stateDerivative(wbt::BlockInformation */*blockInfo*/, wbt::Error */*error*/) { return true; }

bool wbt::Block::initializeInitialConditions(wbt::BlockInformation */*blockInfo*/, wbt::Error */*error*/) { return true; }
