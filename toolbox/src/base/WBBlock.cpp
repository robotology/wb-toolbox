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
#include "Parameter.h"
#include "RobotInterface.h"
#include "Signal.h"
#include "WholeBodySingleton.h"

#include <Eigen/Core>
#include <iDynTree/Core/AngularMotionVector3.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/LinearMotionVector3.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/Core/Twist.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/VectorFixSize.h>
#include <iDynTree/KinDynComputations.h>

#include <array>
#include <memory>
#include <ostream>

using namespace wbt;

enum ParamIndex
{
    Bias = Block::NumberOfParameters - 1,
    WBStruct,
    ConfBlockName
};

/**
 * @brief Container for data structures used for using
 * `iDynTree::KinDynComputations::setRobotState`
 */
struct WBBlock::iDynTreeRobotState
{
    iDynTree::Twist baseVelocity;
    iDynTree::Vector3 gravity;
    iDynTree::Transform world_T_base;
    iDynTree::VectorDynSize jointsVelocity;
    iDynTree::VectorDynSize jointsPosition;
    iDynTreeRobotState(const unsigned& dofs = 0, const std::array<double, 3>& gravity = {});
};

WBBlock::iDynTreeRobotState::iDynTreeRobotState(const unsigned& dofs,
                                                const std::array<double, 3>& gravity)
    : gravity(gravity.data(), 3)
    , jointsVelocity(dofs)
    , jointsPosition(dofs)
{
    jointsPosition.zero();
    jointsVelocity.zero();
}

std::shared_ptr<iDynTree::KinDynComputations> WBBlock::getKinDynComputations() const
{
    return m_robotInterface->getKinDynComputations();
}

const std::shared_ptr<wbt::RobotInterface> WBBlock::getRobotInterface() const
{
    return m_robotInterface;
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

    if (!m_robotState) {
        wbtError << "Failed to access iDynTreeRobotState object.";
        return false;
    }

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
        fromEigen(m_robotState->world_T_base, Matrix4dSimulink(buffer));
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
            m_robotState->jointsPosition.setVal(i, buffer[i]);
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
        m_robotState->baseVelocity = Twist(LinVelocity(buffer, 3), AngVelocity(buffer + 3, 3));
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
            m_robotState->jointsVelocity.setVal(i, buffer[i]);
        }
    }

    // UPDATE THE IDYNTREE ROBOT STATE WITH NEW DATA
    // =============================================

    if (!kinDyn) {
        wbtError << "Failed to access the KinDynComputations object.";
        return false;
    }

    bool ok = kinDyn->setRobotState(m_robotState->world_T_base,
                                    m_robotState->jointsPosition,
                                    m_robotState->baseVelocity,
                                    m_robotState->jointsVelocity,
                                    m_robotState->gravity);

    if (!ok) {
        wbtError << "Failed to set the iDynTree robot state.";
        return false;
    }

    return true;
}

WBBlock::WBBlock()
    : m_robotState{nullptr}
{}

WBBlock::~WBBlock() = default;

unsigned WBBlock::numberOfParameters()
{
    return WBBlock::NumberOfParameters;
}

bool WBBlock::parseParameters(BlockInformation* blockInfo)
{
    const auto DynPar = ParameterMetadata::DynamicSize;

    const std::vector<ParameterMetadata> metadata{
        {ParameterType::STRUCT_STRING, ParamIndex::WBStruct, 1, 1, "RobotName"},
        {ParameterType::STRUCT_STRING, ParamIndex::WBStruct, 1, 1, "UrdfFile"},
        {ParameterType::STRUCT_CELL_STRING, ParamIndex::WBStruct, 1, DynPar, "ControlledJoints"},
        {ParameterType::STRUCT_CELL_STRING, ParamIndex::WBStruct, 1, DynPar, "ControlBoardsNames"},
        {ParameterType::STRUCT_STRING, ParamIndex::WBStruct, 1, 1, "LocalName"},
        {ParameterType::STRUCT_DOUBLE, ParamIndex::WBStruct, 1, 3, "GravityVector"},
        {ParameterType::STRING, ParamIndex::ConfBlockName, 1, 1, "ConfBlockName"}};

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            wbtError << "Failed to store parameter metadata";
            return false;
        }
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

    // Despite after this configureSizeAndPorts step this object will be deleted, in this step many
    // blocks need to know configuration-dependent information such as the DoFs. We store now a
    // RobotInterface object and then again in the initialize() method.

    // Ask the WholeBodySingleton to create the RobotInterface object. It will hold a weak pointer,
    // returning a shared pointer that is stored here in WBBlock. This WBBlock object and all other
    // WBBlocks that share the same RobotInterface own its memory.
    m_robotInterface = WholeBodySingleton::sharedInstance().storeConfiguration(m_parameters);

    if (!m_robotInterface) {
        wbtError << "Failed to get the RobotInterface object from the WholeBodySingleton.";
        return false;
    }

    // Check if the DoFs are positive
    if (m_robotInterface->getConfiguration().getNumberOfDoFs() < 1) {
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

    // Ask the WholeBodySingleton to create the RobotInterface object. It will hold a weak pointer,
    // returning a shared pointer that is stored here in WBBlock. This WBBlock object and all other
    // WBBlocks that share the same RobotInterface own its memory.
    m_robotInterface = WholeBodySingleton::sharedInstance().storeConfiguration(m_parameters);

    if (!m_robotInterface) {
        wbtError << "Failed to get the RobotInterface object from the WholeBodySingleton.";
        return false;
    }

    // Check if the DoFs are positive
    const auto dofs = m_robotInterface->getConfiguration().getNumberOfDoFs();
    if (dofs < 1) {
        wbtError << "Failed to configure WBBlock. Read 0 DoFs.";
        return false;
    }

    // Initialize the m_robotState member
    m_robotState.reset(
        new iDynTreeRobotState(dofs, m_robotInterface->getConfiguration().getGravityVector()));

    if (!m_robotState) {
        wbtError << "Failed to initialize the iDynTreeRobotState object";
        return false;
    }

    return true;
}
