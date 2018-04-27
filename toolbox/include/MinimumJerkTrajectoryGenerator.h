/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_MINJERKTRAJGENERATOR_H
#define WBT_MINJERKTRAJGENERATOR_H

#include "Block.h"
#include <memory>
#include <yarp/sig/Vector.h>

namespace wbt {
    class MinimumJerkTrajectoryGenerator;
    class BlockInformation;
} // namespace wbt

namespace iCub {
    namespace ctrl {
        class minJerkTrajGen;
    }
} // namespace iCub

/**
 * @brief The wbt::MinimumJerkTrajectoryGenerator class
 *
 * @section Parameters
 *
 * In addition to @ref block_parameters, wbt::MinimumJerkTrajectoryGenerator requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::DOUBLE | 0 + Block::NumberOfParameters | 1 | 1 | "SampleTime"                |
 * | ::DOUBLE | 1 + Block::NumberOfParameters | 1 | 1 | "SettlingTime"              |
 * | ::BOOL   | 2 + Block::NumberOfParameters | 1 | 1 | "ComputeFirstDerivative"    |
 * | ::BOOL   | 3 + Block::NumberOfParameters | 1 | 1 | "ComputeSecondDerivative"   |
 * | ::BOOL   | 4 + Block::NumberOfParameters | 1 | 1 | "ReadInitialValue"          |
 * | ::BOOL   | 5 + Block::NumberOfParameters | 1 | 1 | "ReadExternalSettlingTime"  |
 * | ::BOOL   | 6 + Block::NumberOfParameters | 1 | 1 | "ResetOnSettlingTimeChange" |
 *
 */
class wbt::MinimumJerkTrajectoryGenerator : public wbt::Block
{
private:
    std::unique_ptr<iCub::ctrl::minJerkTrajGen> m_generator;

    int m_outputFirstDerivativeIndex = -1;
    int m_outputSecondDerivativeIndex = -1;

    double m_previousSettlingTime;

    bool m_firstRun = true;
    bool m_readInitialValue = false;
    bool m_readExternalSettlingTime = false;
    bool m_resetOnSettlingTimeChange = false;
    std::unique_ptr<yarp::sig::Vector> m_initialValues;
    std::unique_ptr<yarp::sig::Vector> m_reference;

public:
    static const std::string ClassName;

    MinimumJerkTrajectoryGenerator();
    ~MinimumJerkTrajectoryGenerator() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_MINJERKTRAJGENERATOR_H
