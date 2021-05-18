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
#include <OsqpEigen/OsqpEigen.h>
#include <OsqpEigen/Constants.hpp>
#include <osqp/util.h>


#include <ostream>
#include <tuple>

#ifdef ERROR
#undef ERROR
#endif

using namespace blockfactory::core;

const unsigned MaxIterations = 100;

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
};

// BLOCK CLASS
// ===========

wbt::block::OSQP::OSQP()
    : pImpl{new impl()}
{}

wbt::block::OSQP::~OSQP() = default;

unsigned wbt::block::OSQP::numberOfParameters()
{
    return Block::numberOfParameters() + 6;
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
    };

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            //bfError << "Failed to store parameter metadata";
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
        ////bfError << "Failed to parse parameters.";
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
        ////bfError << "Failed to get parameters after their parsing.";
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
        //bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool wbt::block::OSQP::solverInitialization(const BlockInformation*blockInfo)
{
    // Check the hessian size
    const auto size_H = blockInfo->getInputPortMatrixSize(InputIndex::Hessian);
    const auto numberOfVariables = size_H.rows;
    if (size_H.rows != size_H.cols) {
        //bfError << "The Hessian matrix should be square.";
        return false;
    }

    // Check the gradient size
    const auto size_g = blockInfo->getInputPortWidth(InputIndex::Gradient);
    if (size_g != numberOfVariables) {
        //bfError << "The gradient size does not match with the Hessian size.";
        return false;
    }

    // Check the simple bounds size
    bool ok = true;
    if (pImpl->useLb) {
        ok = ok && (blockInfo->getInputPortWidth(InputIndex_lb) == numberOfVariables);
    }
    if (pImpl->useUb) {
        ok = ok && (blockInfo->getInputPortWidth(InputIndex_ub) == numberOfVariables);
    }
    if (!ok) {
        //bfError << "Sizes of bounds do not match with the number of variables.";
        return false;
    }

    int numberOfTotalConstraints = 0;

    if (pImpl->useLbA || pImpl->useUbA) {
        // Check the constraints size
        const auto size_c = blockInfo->getInputPortMatrixSize(InputIndex_constraints);
        int numberOfConstraints = size_c.rows;
        numberOfTotalConstraints += size_c.rows;
        if (size_c.cols != numberOfVariables) {
            //bfError << "The column size of the constraints matrix does not match with "
            //        << "the Hessian size";
            return false;
        }

        // Resize the buffer
        //pImpl->constraints_rowMajor.resize(size_c.rows, size_c.cols);

        // Check the constraints' bound size
        bool ok = true;
        if (pImpl->useLbA) {
            ok = ok && (blockInfo->getInputPortWidth(InputIndex_lbA) == numberOfConstraints);
        }
        if (pImpl->useUbA) {
            ok = ok && (blockInfo->getInputPortWidth(InputIndex_ubA) == numberOfConstraints);
        }
        if (!ok) {
            bfError << "Sizes of constraints' bounds do not match with the number of constraints.";
            return false;
        }
    }

    if (pImpl->useLb || pImpl->useUb) {
        numberOfTotalConstraints += numberOfVariables;
    }

    // Allocate the solver.
    pImpl->sqSolver = std::unique_ptr<OsqpEigen::Solver>(
        new OsqpEigen::Solver());

    // In qpOASES, SQProblem is used also without constraints, while other classes (e.g. QProblem, QProblemB) assume
    // fixed H and g and we cannot use them since we assume them as time varying quantities. For the time being, we
    // keep the same structure also for OSQP
    pImpl->sqSolver->data()->setNumberOfVariables(numberOfVariables);

    pImpl->sqSolver->data()->setNumberOfConstraints(numberOfTotalConstraints);

    if (!pImpl->sqSolver) {
        bfError << "Failed to allocate the OsqpEigen::Solver object.";
        return false;
    }

    // Setup options
    pImpl->sqSolver->settings()->setVerbosity(true);
    // Remove internal state
    pImpl->sqSolver->settings()->setAdaptiveRho(false);
    pImpl->sqSolver->settings()->setWarmStart(true);
}

bool wbt::block::OSQP::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!OSQP::parseParameters(blockInfo)) {
        //bfError << "Failed to parse parameters.";
        return false;
    }

    bool ok = true;
    ok = ok && m_parameters.getParameter("UseLbA", pImpl->useLbA);
    ok = ok && m_parameters.getParameter("UseUbA", pImpl->useUbA);
    ok = ok && m_parameters.getParameter("UseLb", pImpl->useLb);
    ok = ok && m_parameters.getParameter("UseUb", pImpl->useUb);
    ok = ok && m_parameters.getParameter("ComputeObjVal", pImpl->computeObjVal);
    ok = ok && m_parameters.getParameter("StopWhenFails", pImpl->stopWhenFails);

    if (!ok) {
        bfError << "Failed to get parameters after their parsing.";
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
    static int initRuns = 0;
    initRuns++;

    std::cerr << "wbt::block::OSQP::initializeInitialConditions run " << initRuns << std::endl;
    pImpl->sqSolver->clearSolver();
    return true;
}


bool wbt::block::OSQP::output(const BlockInformation* blockInfo)
{
    static int outputRuns = 0;
    outputRuns++;



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

    pImpl->sqSolver->data()->setNumberOfVariables(blockInfo->getInputPortMatrixSize(InputIndex::Hessian).rows);

    Eigen::Map<Eigen::VectorXd> gradient_colMajor(
        const_cast<double*>(gradientSignal->getBuffer<double>()),
        blockInfo->getInputPortWidth(InputIndex::Gradient));

    if (!hessianSignal || !gradientSignal) {
        bfError << "Input signals not valid.";
        return false;
    }

    int numberOfVariables = blockInfo->getInputPortMatrixSize(InputIndex::Hessian).rows;
    int nrOfTotalConstraints = 0;
    int numberOfProperConstraints = 0;

    Eigen::VectorXd properConstraintsLowerBounds;
    Eigen::VectorXd properConstraintsUpperBounds;
    Eigen::VectorXd variableConstraintsLowerBounds;
    Eigen::VectorXd variableConstraintsUpperBounds;
    Eigen::MatrixXd properConstraintMatrix;

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

        numberOfProperConstraints = blockInfo->getInputPortMatrixSize(InputIndex_constraints).rows;
        nrOfTotalConstraints += numberOfProperConstraints;
        properConstraintMatrix = constraints_colMajor;

        if (pImpl->useLbA) {
            InputSignalPtr lbASignal = blockInfo->getInputPortSignal(InputIndex_lbA);
            if (!lbASignal) {
                bfError << "Signal for lbA is not valid.";
                return false;
            }
            //std::cerr << "blockInfo->getInputPortWidth(InputIndex_lbA):" << blockInfo->getInputPortWidth(InputIndex_lbA) << std::endl;
            Eigen::Map<Eigen::VectorXd> lbA_colMajor(
                const_cast<double*>(lbASignal->getBuffer<double>()),
                blockInfo->getInputPortWidth(InputIndex_lbA));
            properConstraintsLowerBounds = lbA_colMajor;
        }

        if (pImpl->useUbA) {
            InputSignalPtr ubASignal = blockInfo->getInputPortSignal(InputIndex_ubA);
            if (!ubASignal) {
                bfError << "Signal for ubA is not valid.";
                return false;
            }
            //std::cerr << "blockInfo->getInputPortWidth(InputIndex_ubA):" << blockInfo->getInputPortWidth(InputIndex_ubA) << std::endl;
            Eigen::Map<Eigen::VectorXd> ubA_colMajor(
                const_cast<double*>(ubASignal->getBuffer<double>()),
                blockInfo->getInputPortWidth(InputIndex_ubA));
            properConstraintsUpperBounds = ubA_colMajor;
        }
    }

    if (pImpl->useLb || pImpl->useUb) {
        // If variables constraints are given, we need to increase the total number of constraints
        nrOfTotalConstraints += numberOfVariables;

        if (pImpl->useLb) {
            InputSignalPtr lbSignal = blockInfo->getInputPortSignal(InputIndex_lb);
            if (!lbSignal) {
                bfError << "Signal for lb is not valid.";
                return false;
            }
            Eigen::Map<const Eigen::VectorXd> variableConstraintsLowerBoundsMap(lbSignal->getBuffer<double>(),
                                                                                lbSignal->getWidth());
            variableConstraintsLowerBounds = variableConstraintsLowerBoundsMap;
        }

        if (pImpl->useUb) {
            InputSignalPtr ubSignal = blockInfo->getInputPortSignal(InputIndex_ub);
            if (!ubSignal) {
                bfError << "Signal for ub is not valid.";
                return false;
            }
            Eigen::Map<const Eigen::VectorXd> variableConstraintsUpperBoundsMap(ubSignal->getBuffer<double>(),
                                                                                ubSignal->getWidth());
            variableConstraintsUpperBounds = variableConstraintsUpperBoundsMap;
        }
    }


    // Let's build the actual constraints passed to OSQP
    if(nrOfTotalConstraints != 0) {
        Eigen::VectorXd totalConstraintsLowerBounds = Eigen::VectorXd::Constant(nrOfTotalConstraints, -WBT_OSQP_INF);
        Eigen::VectorXd totalConstraintsUpperBounds = Eigen::VectorXd::Constant(nrOfTotalConstraints, WBT_OSQP_INF);
        Eigen::MatrixXd totalConstraintsMatrix = Eigen::MatrixXd(nrOfTotalConstraints, numberOfVariables);

        if (pImpl->useLbA) {
            totalConstraintsLowerBounds.head(numberOfProperConstraints) = properConstraintsLowerBounds;
        }

        if (pImpl->useUbA) {
            totalConstraintsUpperBounds.head(numberOfProperConstraints) = properConstraintsUpperBounds;
        }


        if (pImpl->useLb) {
            totalConstraintsLowerBounds.tail(numberOfVariables) = variableConstraintsLowerBounds;
        }

        if (pImpl->useUb) {
            totalConstraintsUpperBounds.tail(numberOfVariables) = variableConstraintsUpperBounds;
        }

        if (pImpl->useLbA || pImpl->useUbA) {
            totalConstraintsMatrix.topRows(numberOfProperConstraints) = properConstraintMatrix;
        }

        if (pImpl->useLb || pImpl->useUb) {
            totalConstraintsMatrix.bottomRows(numberOfVariables) = Eigen::MatrixXd::Identity(numberOfVariables, numberOfVariables);
        }

        if (!pImpl->sqSolver->isInitialized()) {
            Eigen::SparseMatrix<double> totalConstraintsMatrixSparse = totalConstraintsMatrix.sparseView();
            pImpl->sqSolver->data()->setLinearConstraintsMatrix(totalConstraintsMatrixSparse);
            pImpl->sqSolver->data()->setBounds(totalConstraintsLowerBounds, totalConstraintsUpperBounds);
        } else {
            Eigen::SparseMatrix<double> totalConstraintsMatrixSparse = totalConstraintsMatrix.sparseView();
            pImpl->sqSolver->updateLinearConstraintsMatrix(totalConstraintsMatrixSparse);
            pImpl->sqSolver->updateBounds(totalConstraintsLowerBounds, totalConstraintsUpperBounds);
        }
    }
    // OUTPUTS
    // =======

    OutputSignalPtr solutionSignal = blockInfo->getOutputPortSignal(OutputIndex::PrimalSolution);
    if (!solutionSignal) {
        //bfError << "Output signal not valid.";
        return false;
    }
    Eigen::Map<Eigen::VectorXd> solution_colMajor(
        const_cast<double*>(solutionSignal->getBuffer<double>()),
        blockInfo->getOutputPortWidth(OutputIndex::PrimalSolution));
    //std::cerr << " blockInfo->ge(OutputIndex::PrimalSolution): " <<  blockInfo->getOutputPortWidth(OutputIndex::PrimalSolution) << std::endl;
    //std::cerr << " solutionSignal->getWidth() " <<  solutionSignal->getWidth() << std::endl;



    OutputSignalPtr statusSignal = blockInfo->getOutputPortSignal(OutputIndex::Status);
    if (!statusSignal) {
        //bfError << "Status signal not valid.";
        return false;
    }
    Eigen::Map<Eigen::VectorXd> status_colMajor(
        const_cast<double*>(statusSignal->getBuffer<double>()),
        blockInfo->getInputPortWidth(OutputIndex::Status));

    // Set inputs
    Eigen::SparseMatrix<double> hessian_sparse = hessian_colMajor.sparseView();
    //std::cerr << "Size of gradient " << gradient_colMajor.size() << std::endl;
    //std::cerr << "Size of gradient " << gradient_colMajor.size() << std::endl;

    if (!pImpl->sqSolver->isInitialized()) {
        pImpl->sqSolver->data()->setHessianMatrix(hessian_sparse);
        //std::cerr << "Size of gradient " << gradient_colMajor.size() << std::endl;
        pImpl->sqSolver->data()->setGradient(gradient_colMajor);
    } else {
        pImpl->sqSolver->updateHessianMatrix(hessian_sparse);
        //std::cerr << "Size of gradient " << gradient_colMajor.size() << std::endl;
        pImpl->sqSolver->updateGradient(gradient_colMajor);
    }


    // Solve
    //std::cerr << "initSolver running" << std::endl;
    if(!pImpl->sqSolver->isInitialized()) {
        if(!pImpl->sqSolver->initSolver()) {
            bfError << "OSQP: initSolver() failed.";
            return false;
        }
    }


    if(!pImpl->sqSolver->solve()) {
        bfError << "OSQP: solve() failed.";
        return false;
    }

    // Get outputs
    solution_colMajor = pImpl->sqSolver->getSolution();

    //std::cerr << "solution_colMajor: " << solution_colMajor << std::endl;

    // Set status (TODO check failure)
    if (!statusSignal->set(0, 0)) {
        //bfError << "Failed to set status signal.";
        return false;
    }

    // OPTIONAL OUTPUTS
    // ================

    if (pImpl->computeObjVal) {
        //bfError << "OSQP: ComputeObjVal options currently not supported.";
        return false;

        /*
        const auto objVal = pImpl->sqProblem->getObjVal();

        OutputSignalPtr objValSignal = blockInfo->getOutputPortSignal(OutputIndex_objVal);
        if (!objValSignal) {
            //bfError << "Object Value signal not valid.";
            return false;
        }

        if (!objValSignal->set(0, objVal)) {
            //bfError << "Failed to set object value signal.";
            return false;
        }*/
    }

    return true;
}
