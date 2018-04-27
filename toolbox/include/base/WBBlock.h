/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_WBBLOCK_H
#define WBT_WBBLOCK_H

#include "Block.h"
#include <memory>

namespace wbt {
    class WBBlock;
    class Signal;
    class Configuration;
    class BlockInformation;
    class RobotInterface;
} // namespace wbt

namespace iDynTree {
    class MatrixDynSize;
    class KinDynComputations;
} // namespace iDynTree

/**
 * @brief Extension of wbt::Block for simplifying the development of whole-body blocks
 *
 * This class provides support of parsing the parameters for creating a wbt::RobotInterface object,
 * and helpers for retrieving iDynTree::KinDynComputations and wbt::RobotInterface objects.
 *
 * @see wbt::Block
 *
 * @section wbblock_parameters WBBlock Parameters
 *
 * In addition to @ref block_parameters, wbt::WBBlock requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::PARAM_STRUCT_STRING      | 0 + Block::NumberOfParameters | 1 | 1 | "RobotName" |
 * | ::PARAM_STRUCT_STRING      | 0 + Block::NumberOfParameters | 1 | 1 | "UrdfFile" |
 * | ::PARAM_STRUCT_CELL_STRING | 0 + Block::NumberOfParameters | 1 | 1 | "ControlledJoints" |
 * | ::PARAM_STRUCT_CELL_STRING | 0 + Block::NumberOfParameters | 1 | 1 | "ControlBoardsNames" |
 * | ::PARAM_STRUCT_STRING      | 0 + Block::NumberOfParameters | 1 | 1 | "LocalName" |
 * | ::PARAM_STRUCT_DOUBLE      | 0 + Block::NumberOfParameters | 1 | 3 | "GravityVector" |
 * | ::PARAM_STRING             | 1 + Block::NumberOfParameters | 1 | 1 | "ConfBlockName" |
 *
 * @note The first set of parameters are fields of the same struct. For this reason they share the
 * same index.
 */
class wbt::WBBlock : public wbt::Block
{
protected:
    struct iDynTreeRobotState;
    std::unique_ptr<iDynTreeRobotState> m_robotState;

    /**
     * @brief Helper for retrieving the iDynTree::KinDynComputations object from
     *        wbt::BlockInformation
     * @param blockInfo A BlockInformation object.
     * @return A pointer to iDynTree::KinDynComputations.
     */
    std::weak_ptr<iDynTree::KinDynComputations>
    getKinDynComputations(const BlockInformation* blockInfo) const;

    /**
     * @brief Helper for retrieving the wbt::RobotInterface object from
     *        wbt::BlockInformation
     * @param blockInfo A BlockInformation object.
     * @return A pointer to wbt::RobotInterface.
     */
    std::weak_ptr<wbt::RobotInterface> getRobotInterface(const BlockInformation* blockInfo) const;

    /**
     * @brief Helper for setting the robot state inside the iDynTree::KinDynComputations object
     *
     * @param basePose The vector containing the base pose.
     * @param jointsPos The vector containing the joints positions.
     * @param baseVelocity The vector containing the base velocity.
     * @param jointsVelocity The vector containing the joints velocities.
     * @param kinDyn A pointer to the block's KinDynComputations object.
     * @return True if success, false otherwise.
     *
     * @see iDynTree::KinDynComputations::setRobotState, wbt::iDynTreeRobotState
     */
    bool setRobotState(const wbt::Signal* basePose,
                       const wbt::Signal* jointsPos,
                       const wbt::Signal* baseVelocity,
                       const wbt::Signal* jointsVelocity,
                       iDynTree::KinDynComputations* kinDyn);

public:
    /// The number of parameters WBBlock requires
    static const unsigned NumberOfParameters;

    WBBlock();
    ~WBBlock() override;
    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
};

#endif // WBT_WBBLOCK_H
