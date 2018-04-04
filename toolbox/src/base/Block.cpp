#include "Block.h"
#include "toolbox.h"

using namespace wbt;

const unsigned Block::NumberOfParameters = 1;

unsigned Block::numberOfParameters()
{
    return Block::NumberOfParameters;
}

std::vector<std::string> Block::additionalBlockOptions()
{
    return std::vector<std::string>();
}

void Block::parameterAtIndexIsTunable(unsigned /*index*/, bool& tunable)
{
    tunable = false;
}

bool Block::checkParameters(const BlockInformation* /*blockInfo*/)
{
    return true;
}

bool Block::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!Block::parseParameters(blockInfo)) {
        wbtError << "Failed to parse Block parameters.";
        return false;
    }

    return true;
}

bool Block::initialize(BlockInformation* blockInfo)
{
    return true;
}

unsigned Block::numberOfDiscreteStates()
{
    return 0;
}

unsigned Block::numberOfContinuousStates()
{
    return 0;
}

bool Block::updateDiscreteState(const BlockInformation* /*blockInfo*/)
{
    return true;
}

bool Block::stateDerivative(const BlockInformation* /*blockInfo*/)
{
    return true;
}

bool Block::initializeInitialConditions(const BlockInformation* /*blockInfo*/)
{
    return true;
}
