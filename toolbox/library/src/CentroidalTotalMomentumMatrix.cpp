/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/CentroidalTotalMomentumMatrix.h"
#include "WBToolbox/Base/Configuration.h"
#include "WBToolbox/Base/RobotInterface.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <BlockFactory/Core/Signal.h>
#include <Eigen/Core>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/MatrixDynSize.h>
#include <iDynTree/Transform.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Indices.h>

#include <memory>
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
    CentroidalTotalMomentumMatrix = 0,
};

// BLOCK PIMPL
// ===========

class CentroidalTotalMomentumMatrix::impl
{
public:
    iDynTree::MatrixDynSize CentroidalTotalMomentumMatrix;
};

// BLOCK CLASS
// ===========

CentroidalTotalMomentumMatrix::CentroidalTotalMomentumMatrix()
    : pImpl{new impl()}
{}

CentroidalTotalMomentumMatrix::~CentroidalTotalMomentumMatrix() = default;

bool CentroidalTotalMomentumMatrix::configureSizeAndPorts(BlockInformation* blockInfo)
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
    // 1) Matrix representing the Centroidal Momentum Matrix (6x(DoFs+6)
    //

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
            {InputIndex::BasePose, Port::Dimensions{4, 4}, Port::DataType::DOUBLE},
            {InputIndex::JointConfiguration, Port::Dimensions{dofs}, Port::DataType::DOUBLE},
        },
        {
            // Outputs
            {OutputIndex::CentroidalTotalMomentumMatrix,
             Port::Dimensions{6, 6 + dofs},
             Port::DataType::DOUBLE},
        });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool CentroidalTotalMomentumMatrix::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    auto kinDyn = getKinDynComputations();
    if (!kinDyn) {
        bfError << "Cannot retrieve handle to KinDynComputations.";
        return false;
    }

    // Initialize buffers
    // ------------------

    // Get the DoFs
    const auto dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    // Output
    pImpl->CentroidalTotalMomentumMatrix.resize(6, 6 + dofs);
    pImpl->CentroidalTotalMomentumMatrix.zero();

    return true;
}

bool CentroidalTotalMomentumMatrix::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool CentroidalTotalMomentumMatrix::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    using MatrixXdSimulink = Matrix<double, Dynamic, Dynamic, Eigen::ColMajor>;
    using MatrixXdiDynTree = Matrix<double, Dynamic, Dynamic, Eigen::RowMajor>;

    // Get the KinDynComputations object
    auto kinDyn = getKinDynComputations();
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
    // Compute the CentroidalTotalMomentumMatrix

    ok = kinDyn->getCentroidalTotalMomentumJacobian(pImpl->CentroidalTotalMomentumMatrix);

    if (!ok) {
        bfError << "Failed to get the Centroidal Total Momentum Matrix .";
        return false;
    }

    // Get the output signal memory location

    OutputSignalPtr output =
        blockInfo->getOutputPortSignal(OutputIndex::CentroidalTotalMomentumMatrix);

    if (!output) {
        bfError << "Output signal not valid.";
        return false;
    }

    // Allocate objects for row-major -> col-major conversion
    Map<MatrixXdiDynTree> CentroidalTotalMomentumMatrixRowMajor =
        toEigen(pImpl->CentroidalTotalMomentumMatrix);
    Map<MatrixXdSimulink> CentroidalTotalMomentumMatrixColMajor(
        output->getBuffer<double>(),
        blockInfo->getOutputPortMatrixSize(OutputIndex::CentroidalTotalMomentumMatrix).rows,
        blockInfo->getOutputPortMatrixSize(OutputIndex::CentroidalTotalMomentumMatrix).cols);

    // Forward the buffer to Simulink transforming it to ColMajor
    CentroidalTotalMomentumMatrixColMajor = CentroidalTotalMomentumMatrixRowMajor;
    return true;
}
