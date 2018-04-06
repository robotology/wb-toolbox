/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBBlock.h"
#include "BlockInformation.h"
#include "Configuration.h"
#include "Log.h"
#include "RobotInterface.h"
#include "Signal.h"
#include "ToolboxSingleton.h"

#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/MatrixFixSize.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <memory>
#include <string>

using namespace wbt;

const unsigned WBBlock::NumberOfParameters = Block::NumberOfParameters + 2;

const unsigned PARAM_IDX_BIAS = Block::NumberOfParameters - 1;
const unsigned ConfigurationParameterIndex = PARAM_IDX_BIAS + 1; // Struct from Simulink
const unsigned ConfBlockNameParameterIndex =
    PARAM_IDX_BIAS + 2; // Absolute name of the block containing the configuration

iDynTreeRobotState::iDynTreeRobotState(const unsigned& dofs, const std::array<double, 3>& gravity)
    : m_gravity(gravity.data(), 3)
    , m_jointsVelocity(dofs)
    , m_jointsPosition(dofs)
{
    m_jointsPosition.zero();
    m_jointsVelocity.zero();
}

std::weak_ptr<iDynTree::KinDynComputations>
WBBlock::getKinDynComputations(const BlockInformation* blockInfo) const
{
    auto robotInterface = getRobotInterface(blockInfo).lock();

    if (!robotInterface) {
        wbtError << "Failed to get the RobotInterface object.";
        return {};
    }

    return robotInterface->getKinDynComputations();
}

std::weak_ptr<wbt::RobotInterface>
WBBlock::getRobotInterface(const BlockInformation* blockInfo) const
{
    auto robotInterface = blockInfo->getRobotInterface();
    if (!robotInterface.lock()) {
        return {};
    }

    return robotInterface;
}

bool WBBlock::setRobotState(const wbt::Signal* basePose,
                            const wbt::Signal* jointsPos,
                            const wbt::Signal* baseVelocity,
                            const wbt::Signal* jointsVelocity,
                            iDynTree::KinDynComputations* kinDyn)
{
    // SAVE THE ROBOT STATE
    // ====================

    using namespace iDynTree;
    using namespace Eigen;
    using Matrix4dSimulink = Matrix<double, 4, 4, Eigen::ColMajor>;

    // Base pose
    // ---------

    if (basePose) {
        // Get the buffer
        double* buffer = basePose->getBuffer<double>();
        if (!buffer) {
            wbtError << "Failed to read the base pose from input port.";
            return false;
        }
        // Fill the data
        fromEigen(m_robotState.m_world_T_base, Matrix4dSimulink(buffer));
    }

    // Joints position
    // ---------------

    if (jointsPos) {
        // Get the buffer
        double* buffer = jointsPos->getBuffer<double>();
        if (!buffer) {
            wbtError << "Failed to read joints positions from input port.";
            return false;
        }
        // Fill the data
        for (unsigned i = 0; i < jointsPos->getWidth(); ++i) {
            m_robotState.m_jointsPosition.setVal(i, buffer[i]);
        }
    }

    // Base Velocity
    // -------------

    if (baseVelocity) {
        // Get the buffer
        double* buffer = baseVelocity->getBuffer<double>();
        if (!buffer) {
            wbtError << "Failed to read the base velocity from input port.";
            return false;
        }
        // Fill the data
        m_robotState.m_baseVelocity = Twist(LinVelocity(buffer, 3), AngVelocity(buffer + 3, 3));
    }

    // Joints velocity
    // ---------------

    if (jointsVelocity) {
        // Get the buffer
        double* buffer = jointsVelocity->getBuffer<double>();
        if (!buffer) {
            wbtError << "Failed to read joints velocities from input port.";
            return false;
        }
        // Fill the data
        for (unsigned i = 0; i < jointsVelocity->getWidth(); ++i) {
            m_robotState.m_jointsVelocity.setVal(i, buffer[i]);
        }
    }

    // UPDATE THE IDYNTREE ROBOT STATE WITH NEW DATA
    // =============================================

    if (!kinDyn) {
        wbtError << "Failed to access the KinDynComputations object.";
        return false;
    }

    bool ok = kinDyn->setRobotState(m_robotState.m_world_T_base,
                                    m_robotState.m_jointsPosition,
                                    m_robotState.m_baseVelocity,
                                    m_robotState.m_jointsVelocity,
                                    m_robotState.m_gravity);

    if (!ok) {
        wbtError << "Failed to set the iDynTree robot state.";
        return false;
    }

    return true;
}

unsigned WBBlock::numberOfParameters()
{
    return WBBlock::NumberOfParameters;
}

bool WBBlock::parseParameters(BlockInformation* blockInfo)
{
    ParameterMetadata fieldRobotNameMedatata(
        ParameterType::STRUCT_STRING, ConfigurationParameterIndex, 1, 1, "RobotName");
    ParameterMetadata fieldUrdfFileMedatata(
        ParameterType::STRUCT_STRING, ConfigurationParameterIndex, 1, 1, "UrdfFile");
    ParameterMetadata fieldControlledJointsMedatata(ParameterType::STRUCT_CELL_STRING,
                                                    ConfigurationParameterIndex,
                                                    1,
                                                    ParameterMetadata::DynamicSize,
                                                    "ControlledJoints");
    ParameterMetadata fieldControlBoardsMedatata(ParameterType::STRUCT_CELL_STRING,
                                                 ConfigurationParameterIndex,
                                                 1,
                                                 ParameterMetadata::DynamicSize,
                                                 "ControlBoardsNames");
    ParameterMetadata fieldLocalNameMedatata(
        ParameterType::STRUCT_STRING, ConfigurationParameterIndex, 1, 1, "LocalName");
    ParameterMetadata fieldGravityVectorMedatata(
        ParameterType::STRUCT_DOUBLE, ConfigurationParameterIndex, 1, 3, "GravityVector");
    ParameterMetadata confBlockNameMedatata(
        ParameterType::STRING, ConfBlockNameParameterIndex, 1, 1, "ConfBlockName");

    // Add the struct into the block information
    bool ok = true;
    ok = ok && blockInfo->addParameterMetadata(fieldRobotNameMedatata);
    ok = ok && blockInfo->addParameterMetadata(fieldUrdfFileMedatata);
    ok = ok && blockInfo->addParameterMetadata(fieldLocalNameMedatata);
    ok = ok && blockInfo->addParameterMetadata(fieldControlledJointsMedatata);
    ok = ok && blockInfo->addParameterMetadata(fieldControlBoardsMedatata);
    ok = ok && blockInfo->addParameterMetadata(fieldGravityVectorMedatata);
    //
    ok = ok && blockInfo->addParameterMetadata(confBlockNameMedatata);

    if (!ok) {
        wbtError << "Failed to store parameters metadata.";
        return false;
    }

    return blockInfo->parseParameters(m_parameters);
}

bool WBBlock::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // Parse the parameters
    if (!WBBlock::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    // Get the RobotInterface containing the Configuration object
    auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }

    // Check if the DoFs are positive
    if (robotInterface->getConfiguration().getNumberOfDoFs() < 1) {
        wbtError << "Failed to configure WBBlock. Read 0 DoFs.";
        return false;
    }

    return true;
}

bool WBBlock::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // Parse the parameters
    if (!WBBlock::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
        return false;
    }

    // Get the RobotInterface object
    auto robotInterface = getRobotInterface(blockInfo).lock();
    if (!robotInterface) {
        wbtError << "RobotInterface has not been correctly initialized.";
        return false;
    }

    // Initialize the m_robotState member
    const unsigned& dofs = robotInterface->getConfiguration().getNumberOfDoFs();
    m_robotState = iDynTreeRobotState(dofs, robotInterface->getConfiguration().getGravityVector());

    return true;
}
