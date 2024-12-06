/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/MassMatrix.h"
#include "WBToolbox/Base/Configuration.h"
#include "WBToolbox/Base/RobotInterface.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Signal.h>
#include <Eigen/Core>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <ostream>
#include <tuple>

using namespace wbt::block;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum InputIndex
{
    BasePose = 0,
    JointConfiguration,
};

enum OutputIndex
{
    MassMatrix = 0,
};

// BLOCK PIMPL
// ===========

class MassMatrix::impl
{
public:
    iDynTree::MatrixDynSize massMatrix;
};

// BLOCK CLASS
// ===========

MassMatrix::MassMatrix()
    : pImpl{new impl()}
{}

MassMatrix::~MassMatrix() = default;

bool MassMatrix::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // Get the DoFs
    const int dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    // INPUTS
    // ======
    //
    // 1) Homogeneous transform for base pose wrt the world frame (4x4 matrix)
    // 2) Joints position (1xDoFs vector)
    //
    // OUTPUTS
    // =======
    //
    // 1) Matrix representing the mass matrix (DoFs+6)x(DoFs+6)
    //

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
            {InputIndex::BasePose, Port::Dimensions{4, 4}, Port::DataType::DOUBLE},
            {InputIndex::JointConfiguration, Port::Dimensions{dofs}, Port::DataType::DOUBLE},
        },
        {
            // Outputs
            {OutputIndex::MassMatrix, Port::Dimensions{dofs + 6, dofs + 6}, Port::DataType::DOUBLE},
        });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool MassMatrix::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    // Initialize buffers
    // ------------------

    // Get the DoFs
    const auto dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    pImpl->massMatrix.resize(6 + dofs, 6 + dofs);
    pImpl->massMatrix.zero();

    return true;
}

bool MassMatrix::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool MassMatrix::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    using namespace iDynTree;
    using MatrixXdSimulink = Matrix<double, Dynamic, Dynamic, Eigen::ColMajor>;
    using MatrixXdiDynTree = Matrix<double, Dynamic, Dynamic, Eigen::RowMajor>;

    // Get the KinDynComputations object
    auto kinDyn = getKinDynComputations();
    if (!kinDyn) {
        bfError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    if (!kinDyn) {
        bfError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    // GET THE SIGNALS POPULATE THE ROBOT STATE
    // ========================================

    InputSignalPtr basePoseSig = blockInfo->getInputPortSignal(InputIndex::BasePose);
    InputSignalPtr jointsPosSig = blockInfo->getInputPortSignal(InputIndex::JointConfiguration);

    if (!basePoseSig || !jointsPosSig) {
        bfError << "Input signals not valid.";
        return false;
    }

    bool ok = setRobotState(basePoseSig, jointsPosSig, nullptr, nullptr, kinDyn.get());

    if (!ok) {
        bfError << "Failed to set the robot state.";
        return false;
    }

    // OUTPUT
    // ======

    // Compute the Mass Matrix
    kinDyn->getFreeFloatingMassMatrix(pImpl->massMatrix);

    // Get the output signal memory location
    OutputSignalPtr output = blockInfo->getOutputPortSignal(OutputIndex::MassMatrix);
    if (!output) {
        bfError << "Output signal not valid.";
        return false;
    }

    // Allocate objects for row-major -> col-major conversion
    Map<MatrixXdiDynTree> massMatrixRowMajor = toEigen(pImpl->massMatrix);
    Map<MatrixXdSimulink> massMatrixColMajor(
        output->getBuffer<double>(),
        blockInfo->getOutputPortMatrixSize(OutputIndex::MassMatrix).rows,
        blockInfo->getOutputPortMatrixSize(OutputIndex::MassMatrix).cols);

    // Forward the buffer to Simulink transforming it to ColMajor
    massMatrixColMajor = massMatrixRowMajor;
    return true;
}
