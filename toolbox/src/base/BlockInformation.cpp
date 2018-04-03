#include "BlockInformation.h"
#include "Parameters.h"

const std::string wbt::BlockOptionPrioritizeOrder = "wbt.BlockOptionPrioritizeOrder";

bool wbt::BlockInformation::optionFromKey(const std::string& /*key*/, double& /*option*/) const
{
    return false;
}
