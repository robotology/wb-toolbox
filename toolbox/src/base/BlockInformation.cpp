#include "BlockInformation.h"

const std::string wbt::BlockOptionPrioritizeOrder = "wbt.BlockOptionPrioritizeOrder";

wbt::BlockInformation::~BlockInformation() {}
bool wbt::BlockInformation::optionFromKey(const std::string& key, wbt::Data &option) const { return false; }
