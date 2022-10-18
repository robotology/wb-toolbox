/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/OSQP.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <BlockFactory/Core/Signal.h>
#include <Eigen/Core>
#include <OsqpEigen/Constants.hpp>
#include <OsqpEigen/OsqpEigen.h>
#include <osqp/util.h>

#include <ostream>
#include <tuple>

#ifdef ERROR
#undef ERROR
#endif

using namespace blockfactory::core;

#define WBT_OSQP_INF 1e8

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = Block::NumberOfParameters - 1,
    UseLbA,
    UseUbA,
    UseLb,
    UseUb,
    ComputeObjVal,
    StopWhenFails,
    AdaptiveRho,
    Polish,
    MaxIterations,
};

enum InputIndex
{
    Hessian = 0,
    Gradient,
    // Other optional inputs
};

static size_t InputIndex_constraints = InputIndex::Gradient;
static size_t InputIndex_lbA = InputIndex::Gradient;
static size_t InputIndex_ubA = InputIndex::Gradient;
static size_t InputIndex_lb = InputIndex::Gradient;
static size_t InputIndex_ub = InputIndex::Gradient;

enum OutputIndex
{
    PrimalSolution = 0,
    Status,
    // Other optional inputs
};

static size_t OutputIndex_objVal = OutputIndex::Status;

// BLOCK PIMPL
// ===========

using MatrixXdSimulink = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;

class wbt::block::OSQP::impl
{
public:
    std::unique_ptr<OsqpEigen::Solver> sqSolver;
    bool useLbA;
    bool useUbA;
    bool useLb;
    bool useUb;
    bool computeObjVal;
    bool stopWhenFails;
    bool adaptiveRho;
    bool polish;
    unsigned maxIterations;
    // Buffers
    size_t numberOfVariables;
    size_t numberOfTotalConstraints;
    size_t numberOfProperConstraints;
    Eigen::VectorXd properConstraintsLowerBounds;
    Eigen::VectorXd properConstraintsUpperBounds;
    Eigen::MatrixXd properConstraintMatrix;
    Eigen::SparseMatrix<double> properConstraintMatrixSparse;
    Eigen::VectorXd variableConstraintsLowerBounds;
    Eigen::VectorXd variableConstraintsUpperBounds;
    Eigen::VectorXd totalConstraintsLowerBounds;
    Eigen::VectorXd totalConstraintsUpperBounds;
    Eigen::MatrixXd totalConstraintsMatrix;
    Eigen::SparseMatrix<double> totalConstraintsMatrixSparse;
};

// BLOCK CLASS
// ===========

wbt::block::OSQP::OSQP()
    : pImpl{new impl()}
{}

wbt::block::OSQP::~OSQP() = default;

unsigned wbt::block::OSQP::numberOfParameters()
{
    return Block::numberOfParameters() + 9;
}

bool wbt::block::OSQP::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> metadata{
        {ParameterType::BOOL, ParamIndex::UseLbA, 1, 1, "UseLbA"},
        {ParameterType::BOOL, ParamIndex::UseUbA, 1, 1, "UseUbA"},
        {ParameterType::BOOL, ParamIndex::UseLb, 1, 1, "UseLb"},
        {ParameterType::BOOL, ParamIndex::UseUb, 1, 1, "UseUb"},
        {ParameterType::BOOL, ParamIndex::ComputeObjVal, 1, 1, "ComputeObjVal"},
        {ParameterType::BOOL, ParamIndex::StopWhenFails, 1, 1, "StopWhenFails"},
        {ParameterType::BOOL, ParamIndex::AdaptiveRho, 1, 1, "AdaptiveRho"},
        {ParameterType::BOOL, ParamIndex::Polish, 1, 1, "Polish"},
        {ParameterType::INT, ParamIndex::MaxIterations, 1, 1, "MaxIterations"},
    };

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            bfError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool wbt::block::OSQP::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // PARAMETERS
    // ==========

    if (!OSQP::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    bool useLbA = false;
    bool useUbA = false;
    bool useLb = false;
    bool useUb = false;
    bool computeObjVal = false;

    bool ok = true;
    ok = ok && m_parameters.getParameter("UseLbA", useLbA);
    ok = ok && m_parameters.getParameter("UseUbA", useUbA);
    ok = ok && m_parameters.getParameter("UseLb", useLb);
    ok = ok && m_parameters.getParameter("UseUb", useUb);
    ok = ok && m_parameters.getParameter("ComputeObjVal", computeObjVal);

    if (!ok) {
        bfError << "Failed to get parameters after their parsing.";
        return false;
    }

    // INPUTS
    // ======
    //
    // 1) Hessian Matrix (nV x nV)
    // 2) Gradient vector (1 x nV)
    // 3) Optional: Constraints matrix (nC x nV)
    // 4) Optional: Constraints lower bounds (1 x nV)
    // 5) Optional: Constraints upper bounds (1 x nV)
    // 6) Optional: Variables lower bounds (1 x nC)
    // 7) Optional: Variables upper bounds (1 x nC)
    //
    // OUTPUT
    // ======
    //
    // 1) Primal solution (1 x nV)
    // 2) Status of the qp solver (1x1)
    // 3) Optional: Value of the object function (1x1)

    InputPortsInfo inputPortsInfo;
    OutputPortsInfo outputPortsInfo;

    // Inputs
    inputPortsInfo.push_back({InputIndex::Hessian,
                              Port::Dimensions{Port::DynamicSize, Port::DynamicSize},
                              Port::DataType::DOUBLE});
    inputPortsInfo.push_back(
        {InputIndex::Gradient, Port::Dimensions{Port::DynamicSize}, Port::DataType::DOUBLE});
    outputPortsInfo.push_back(
        {OutputIndex::PrimalSolution, Port::Dimensions{Port::DynamicSize}, Port::DataType::DOUBLE});
    outputPortsInfo.push_back({OutputIndex::Status, Port::Dimensions{1}, Port::DataType::DOUBLE});

    // Optional inputs
    size_t numberOfInputs = InputIndex::Gradient;
    if (useLbA || useUbA) {
        InputIndex_constraints = ++numberOfInputs;
        inputPortsInfo.push_back({InputIndex_constraints,
                                  Port::Dimensions{Port::DynamicSize, Port::DynamicSize},
                                  Port::DataType::DOUBLE});
    }
    if (useLbA) {
        InputIndex_lbA = ++numberOfInputs;
        inputPortsInfo.push_back(
            {InputIndex_lbA, Port::Dimensions{Port::DynamicSize}, Port::DataType::DOUBLE});
    }
    if (useUbA) {
        InputIndex_ubA = ++numberOfInputs;
        inputPortsInfo.push_back(
            {InputIndex_ubA, Port::Dimensions{Port::DynamicSize}, Port::DataType::DOUBLE});
    }
    if (useLb) {
        InputIndex_lb = ++numberOfInputs;
        inputPortsInfo.push_back(
            {InputIndex_lb, Port::Dimensions{Port::DynamicSize}, Port::DataType::DOUBLE});
    }
    if (useUb) {
        InputIndex_ub = ++numberOfInputs;
        inputPortsInfo.push_back(
            {InputIndex_ub, Port::Dimensions{Port::DynamicSize}, Port::DataType::DOUBLE});
    }

    // Optional outputs
    size_t numberOfOutputs = OutputIndex::Status;
    if (computeObjVal) {
        OutputIndex_objVal = ++numberOfOutputs;
        outputPortsInfo.push_back(
            {OutputIndex_objVal, Port::Dimensions{1}, Port::DataType::DOUBLE});
    }

    if (!blockInfo->setPortsInfo(inputPortsInfo, outputPortsInfo)) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool wbt::block::OSQP::solverInitialization(const BlockInformation* blockInfo)
{
    // Check the hessian size
    const auto size_H = blockInfo->getInputPortMatrixSize(InputIndex::Hessian);
    pImpl->numberOfVariables = size_H.rows;
    if (size_H.rows != size_H.cols) {
        bfError << "The Hessian matrix should be square.";
        return false;
    }

    // Check the gradient size
    const auto size_g = blockInfo->getInputPortWidth(InputIndex::Gradient);
    if (size_g != pImpl->numberOfVariables) {
        bfError << "The gradient size does not match with the Hessian size.";
        return false;
    }

    // Check the simple bounds size
    bool ok = true;
    if (pImpl->useLb) {
        ok = ok && (blockInfo->getInputPortWidth(InputIndex_lb) == pImpl->numberOfVariables);
    }
    if (pImpl->useUb) {
        ok = ok && (blockInfo->getInputPortWidth(InputIndex_ub) == pImpl->numberOfVariables);
    }
    if (!ok) {
        bfError << "Sizes of bounds do not match with the number of variables.";
        return false;
    }

    pImpl->numberOfTotalConstraints = 0;
    pImpl->numberOfProperConstraints = 0;
    if (pImpl->useLbA || pImpl->useUbA) {
        // Check the constraints size
        const auto size_c = blockInfo->getInputPortMatrixSize(InputIndex_constraints);
        pImpl->numberOfProperConstraints = size_c.rows;
        pImpl->numberOfTotalConstraints += pImpl->numberOfProperConstraints;
        if (size_c.cols != pImpl->numberOfVariables) {
            bfError << "The column size of the constraints matrix does not match with "
                    << "the Hessian size";
            return false;
        }

        // Check the constraints' bound size
        bool ok = true;
        if (pImpl->useLbA) {
            ok = ok
                 && (blockInfo->getInputPortWidth(InputIndex_lbA)
                     == pImpl->numberOfProperConstraints);
        }
        if (pImpl->useUbA) {
            ok = ok
                 && (blockInfo->getInputPortWidth(InputIndex_ubA)
                     == pImpl->numberOfProperConstraints);
        }
        if (!ok) {
            bfError << "Sizes of constraints' bounds do not match with the number of constraints.";
            return false;
        }
    }

    if (pImpl->useLb || pImpl->useUb) {
        pImpl->numberOfTotalConstraints += pImpl->numberOfVariables;
    }

    // Allocate the solver.
    pImpl->sqSolver = std::make_unique<OsqpEigen::Solver>();

    pImpl->sqSolver->data()->setNumberOfVariables(pImpl->numberOfVariables);
    pImpl->sqSolver->data()->setNumberOfConstraints(pImpl->numberOfTotalConstraints);

    if (!pImpl->sqSolver) {
        bfError << "Failed to allocate the OsqpEigen::Solver object.";
        return false;
    }

    // Resize buffers
    pImpl->totalConstraintsLowerBounds =
        Eigen::VectorXd::Constant(pImpl->numberOfTotalConstraints, -WBT_OSQP_INF);
    pImpl->totalConstraintsUpperBounds =
        Eigen::VectorXd::Constant(pImpl->numberOfTotalConstraints, WBT_OSQP_INF);
    pImpl->totalConstraintsMatrix =
        Eigen::MatrixXd(pImpl->numberOfTotalConstraints, pImpl->numberOfVariables);

    // Setup options
    pImpl->sqSolver->settings()->setVerbosity(false);

    // Setup adaptive_rho
    // Note that enabling adaptive_rho could cause converge problems
    // See https://github.com/oxfordcontrol/osqp/issues/151
    pImpl->sqSolver->settings()->setAdaptiveRho(pImpl->adaptiveRho);

    // Set warm start to true to permit to just update the values
    // of hessians, gradient and constraints
    pImpl->sqSolver->settings()->setWarmStart(true);

    // Setup the polishing option
    pImpl->sqSolver->settings()->setPolish(pImpl->polish);

    // Setup the maximum number of iterations
    pImpl->sqSolver->settings()->setMaxIteration(pImpl->maxIterations);

    return true;
}

bool wbt::block::OSQP::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    int maxIter = -1;

    if (!OSQP::parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    bool ok = true;
    ok = ok && m_parameters.getParameter("UseLbA", pImpl->useLbA);
    ok = ok && m_parameters.getParameter("UseUbA", pImpl->useUbA);
    ok = ok && m_parameters.getParameter("UseLb", pImpl->useLb);
    ok = ok && m_parameters.getParameter("UseUb", pImpl->useUb);
    ok = ok && m_parameters.getParameter("ComputeObjVal", pImpl->computeObjVal);
    ok = ok && m_parameters.getParameter("StopWhenFails", pImpl->stopWhenFails);
    ok = ok && m_parameters.getParameter("AdaptiveRho", pImpl->adaptiveRho);
    ok = ok && m_parameters.getParameter("Polish", pImpl->polish);
    ok = ok && m_parameters.getParameter("MaxIterations", maxIter);

    if (!ok) {
        bfError << "Failed to get parameters after their parsing.";
        return false;
    }

    // Check if the maximum number of Iterations read correctly from Simulink GUI
    if (maxIter > 0) {
        pImpl->maxIterations = maxIter;
    }
    else {
        bfError << "Failed to set the maximum number of iterations.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================
    if (!this->solverInitialization(blockInfo)) {
        return false;
    }

    return true;
}

bool wbt::block::OSQP::initializeInitialConditions(const BlockInformation* /*blockInfo*/)
{
    pImpl->sqSolver->clearSolver();
    return true;
}

bool wbt::block::OSQP::output(const BlockInformation* blockInfo)
{
    // INPUTS
    // ======

    // Get the Signals.
    // Note: the Hessian is symmetric, no need for conversion from column to row major.
    InputSignalPtr hessianSignal = blockInfo->getInputPortSignal(InputIndex::Hessian);
    InputSignalPtr gradientSignal = blockInfo->getInputPortSignal(InputIndex::Gradient);

    Eigen::Map<MatrixXdSimulink> hessian_colMajor(
        const_cast<double*>(hessianSignal->getBuffer<double>()),
        blockInfo->getInputPortMatrixSize(InputIndex::Hessian).rows,
        blockInfo->getInputPortMatrixSize(InputIndex::Hessian).cols);

    pImpl->sqSolver->data()->setNumberOfVariables(
        blockInfo->getInputPortMatrixSize(InputIndex::Hessian).rows);

    Eigen::Map<Eigen::VectorXd> gradient_colMajor(
        const_cast<double*>(gradientSignal->getBuffer<double>()),
        blockInfo->getInputPortWidth(InputIndex::Gradient));

    if (!hessianSignal || !gradientSignal) {
        bfError << "Input signals not valid.";
        return false;
    }

    // OPTIONAL INPUTS
    // ===============
    if (pImpl->useLbA || pImpl->useUbA) {
        InputSignalPtr constraintsSignal = blockInfo->getInputPortSignal(InputIndex_constraints);
        if (!constraintsSignal) {
            bfError << "Signal for lbA is not valid.";
            return false;
        }

        Eigen::Map<MatrixXdSimulink> constraints_colMajor(
            const_cast<double*>(constraintsSignal->getBuffer<double>()),
            blockInfo->getInputPortMatrixSize(InputIndex_constraints).rows,
            blockInfo->getInputPortMatrixSize(InputIndex_constraints).cols);

        pImpl->properConstraintMatrix = constraints_colMajor;

        if (pImpl->useLbA) {
            InputSignalPtr lbASignal = blockInfo->getInputPortSignal(InputIndex_lbA);
            if (!lbASignal) {
                bfError << "Signal for lbA is not valid.";
                return false;
            }

            Eigen::Map<Eigen::VectorXd> lbA_colMajor(
                const_cast<double*>(lbASignal->getBuffer<double>()),
                blockInfo->getInputPortWidth(InputIndex_lbA));
            pImpl->properConstraintsLowerBounds = lbA_colMajor;
        }

        if (pImpl->useUbA) {
            InputSignalPtr ubASignal = blockInfo->getInputPortSignal(InputIndex_ubA);
            if (!ubASignal) {
                bfError << "Signal for ubA is not valid.";
                return false;
            }

            Eigen::Map<Eigen::VectorXd> ubA_colMajor(
                const_cast<double*>(ubASignal->getBuffer<double>()),
                blockInfo->getInputPortWidth(InputIndex_ubA));
            pImpl->properConstraintsUpperBounds = ubA_colMajor;
        }
    }

    if (pImpl->useLb || pImpl->useUb) {
        if (pImpl->useLb) {
            InputSignalPtr lbSignal = blockInfo->getInputPortSignal(InputIndex_lb);
            if (!lbSignal) {
                bfError << "Signal for lb is not valid.";
                return false;
            }
            Eigen::Map<const Eigen::VectorXd> variableConstraintsLowerBoundsMap(
                lbSignal->getBuffer<double>(), lbSignal->getWidth());
            pImpl->variableConstraintsLowerBounds = variableConstraintsLowerBoundsMap;
        }

        if (pImpl->useUb) {
            InputSignalPtr ubSignal = blockInfo->getInputPortSignal(InputIndex_ub);
            if (!ubSignal) {
                bfError << "Signal for ub is not valid.";
                return false;
            }
            Eigen::Map<const Eigen::VectorXd> variableConstraintsUpperBoundsMap(
                ubSignal->getBuffer<double>(), ubSignal->getWidth());
            pImpl->variableConstraintsUpperBounds = variableConstraintsUpperBoundsMap;
        }
    }

    // Let's build the actual constraints passed to OSQP
    if (pImpl->numberOfTotalConstraints != 0) {
        if (pImpl->useLbA) {
            pImpl->totalConstraintsLowerBounds.head(pImpl->numberOfProperConstraints) =
                pImpl->properConstraintsLowerBounds;
        }

        if (pImpl->useUbA) {
            pImpl->totalConstraintsUpperBounds.head(pImpl->numberOfProperConstraints) =
                pImpl->properConstraintsUpperBounds;
        }

        if (pImpl->useLb) {
            pImpl->totalConstraintsLowerBounds.tail(pImpl->numberOfVariables) =
                pImpl->variableConstraintsLowerBounds;
        }

        if (pImpl->useUb) {
            pImpl->totalConstraintsUpperBounds.tail(pImpl->numberOfVariables) =
                pImpl->variableConstraintsUpperBounds;
        }

        if (pImpl->useLbA || pImpl->useUbA) {
            pImpl->totalConstraintsMatrix.topRows(pImpl->numberOfProperConstraints) =
                pImpl->properConstraintMatrix;
        }

        if (pImpl->useLb || pImpl->useUb) {
            pImpl->totalConstraintsMatrix.bottomRows(pImpl->numberOfVariables) =
                Eigen::MatrixXd::Identity(pImpl->numberOfVariables, pImpl->numberOfVariables);
        }

        if (!pImpl->sqSolver->isInitialized()) {
            pImpl->totalConstraintsMatrixSparse = pImpl->totalConstraintsMatrix.sparseView();
            pImpl->sqSolver->data()->setLinearConstraintsMatrix(
                pImpl->totalConstraintsMatrixSparse);
            pImpl->sqSolver->data()->setBounds(pImpl->totalConstraintsLowerBounds,
                                               pImpl->totalConstraintsUpperBounds);
        }
        else {
            pImpl->totalConstraintsMatrixSparse = pImpl->totalConstraintsMatrix.sparseView();
            pImpl->sqSolver->updateLinearConstraintsMatrix(pImpl->totalConstraintsMatrixSparse);
            pImpl->sqSolver->updateBounds(pImpl->totalConstraintsLowerBounds,
                                          pImpl->totalConstraintsUpperBounds);
        }
    }
    // OUTPUTS
    // =======

    OutputSignalPtr solutionSignal = blockInfo->getOutputPortSignal(OutputIndex::PrimalSolution);
    if (!solutionSignal) {
        bfError << "Output signal not valid.";
        return false;
    }
    Eigen::Map<Eigen::VectorXd> solution_colMajor(
        const_cast<double*>(solutionSignal->getBuffer<double>()),
        blockInfo->getOutputPortWidth(OutputIndex::PrimalSolution));

    OutputSignalPtr statusSignal = blockInfo->getOutputPortSignal(OutputIndex::Status);
    if (!statusSignal) {
        bfError << "Status signal not valid.";
        return false;
    }
    Eigen::Map<Eigen::VectorXd> status_colMajor(
        const_cast<double*>(statusSignal->getBuffer<double>()),
        blockInfo->getInputPortWidth(OutputIndex::Status));

    // Set inputs
    Eigen::SparseMatrix<double> hessian_sparse = hessian_colMajor.sparseView();

    if (!pImpl->sqSolver->isInitialized()) {
        pImpl->sqSolver->data()->setHessianMatrix(hessian_sparse);
        pImpl->sqSolver->data()->setGradient(gradient_colMajor);
    }
    else {
        pImpl->sqSolver->updateHessianMatrix(hessian_sparse);
        pImpl->sqSolver->updateGradient(gradient_colMajor);
    }

    // Solve
    if (!pImpl->sqSolver->isInitialized()) {
        if (!pImpl->sqSolver->initSolver()) {
            bfError << "OSQP: initSolver() failed.";
            return false;
        }
    }

    bool solveReturnVal = pImpl->sqSolver->solve();

    if (pImpl->stopWhenFails && !solveReturnVal) {
        bfError << "OSQP: solve() failed.";
        return false;
    }

    // Get outputs
    if (solveReturnVal) {
        solution_colMajor = pImpl->sqSolver->getSolution();
    }

    // Set status
    double status = solveReturnVal ? 0 : 1;
    if (!statusSignal->set(0, status)) {
        bfError << "Failed to set status signal.";
        return false;
    }

    // OPTIONAL OUTPUTS
    // ================

    if (pImpl->computeObjVal) {

        double objVal = pImpl->sqSolver->workspace()->info->obj_val;

        OutputSignalPtr objValSignal = blockInfo->getOutputPortSignal(OutputIndex_objVal);
        if (!objValSignal) {
            bfError << "Object Value signal not valid.";
            return false;
        }

        if (!objValSignal->set(0, objVal)) {
            bfError << "Failed to set object value signal.";
            return false;
        }
    }

    return true;
}
