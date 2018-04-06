/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "BlockInformation.h"
#include "Parameters.h"

const std::string wbt::BlockOptionPrioritizeOrder = "wbt.BlockOptionPrioritizeOrder";

bool wbt::BlockInformation::optionFromKey(const std::string& /*key*/, double& /*option*/) const
{
    return false;
}
