/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "Block.h"

#include <memory>
#include <string>
#include <vector>

#ifndef WBT_FILTER_H
#define WBT_FILTER_H

namespace wbt {
    class DiscreteFilter;
} // namespace wbt

namespace iCub {
    namespace ctrl {
        class IFilter;
    }
} // namespace iCub

namespace yarp {
    namespace sig {
        class Vector;
    }
} // namespace yarp

/**
 * @brief The wbt::DiscreteFilter class
 *
 * @section Parameters
 *
 * In addition to @ref block_parameters, wbt::DiscreteFilter requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "Fc"          |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "Ts"          |
 * | ::STRUCT_INT    | 0 + Block::NumberOfParameters | 1 | 1             | "MedianOrder" |
 * | ::STRUCT_STRING | 0 + Block::NumberOfParameters | 1 | 1             | "FilterType"  |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | ::DynamicSize | "NumCoeffs"   |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | ::DynamicSize | "DenCoeffs"   |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "InitStatus"  |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "y0"          |
 * | ::STRUCT_DOUBLE | 0 + Block::NumberOfParameters | 1 | 1             | "u0"          |
 *
 */
class wbt::DiscreteFilter : public wbt::Block
{
private:
    std::unique_ptr<iCub::ctrl::IFilter> m_filter;
    std::unique_ptr<yarp::sig::Vector> m_y0;
    std::unique_ptr<yarp::sig::Vector> m_u0;
    std::unique_ptr<yarp::sig::Vector> m_inputSignalVector;

public:
    static const std::string ClassName;

    DiscreteFilter();
    ~DiscreteFilter() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool initializeInitialConditions(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_FILTER_H
