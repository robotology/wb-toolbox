/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "QpOases.h"
#include "Core/BlockInformation.h"
#include "Core/Log.h"
#include "Core/Parameter.h"
#include "Core/Parameters.h"
#include "Core/Signal.h"

#include <Eigen/Core>
#include <qpOASES.hpp>

#include <ostream>
#include <tuple>

using namespace wbt;

const unsigned MaxIterations = 100;

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

static int InputIndex_constraints = InputIndex::Gradient;
static int InputIndex_lbA = InputIndex::Gradient;
static int InputIndex_ubA = InputIndex::Gradient;
static int InputIndex_lb = InputIndex::Gradient;
static int InputIndex_ub = InputIndex::Gradient;

enum OutputIndex
{
    PrimalSolution = 0,
    Status,
    // Other optional inputs
};

static int OutputIndex_objVal = OutputIndex::Status;

// BLOCK PIMPL
// ===========

using MatrixXdSimulink = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
using MatrixXdQpOases = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;

class QpOases::impl
{
public:
    std::unique_ptr<qpOASES::SQProblem> sqProblem;
    MatrixXdQpOases constraints_rowMajor;
    bool useLbA;
    bool useUbA;
    bool useLb;
    bool useUb;
    bool computeObjVal;
    bool stopWhenFails;
};

// BLOCK CLASS
// ===========

QpOases::QpOases()
    : pImpl{new impl()}
{}

QpOases::~QpOases() = default;

unsigned QpOases::numberOfParameters()
{
    return Block::numberOfParameters() + 6;
}

bool QpOases::parseParameters(BlockInformation* blockInfo)
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
            wbtError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool QpOases::configureSizeAndPorts(BlockInformation* blockInfo)
{
    // PARAMETERS
    // ==========

    if (!QpOases::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
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
        wbtError << "Failed to get parameters after their parsing.";
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

    BlockInformation::IOData ioData;

    // Inputs
    ioData.input.emplace_back(InputIndex::Hessian,
                              std::vector<int>{Signal::DynamicSize, Signal::DynamicSize},
                              DataType::DOUBLE);
    ioData.input.emplace_back(
        InputIndex::Gradient, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    ioData.output.emplace_back(
        OutputIndex::PrimalSolution, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    ioData.output.emplace_back(OutputIndex::Status, std::vector<int>{1}, DataType::DOUBLE);

    // Optional inputs
    int numberOfInputs = InputIndex::Gradient;
    if (useLbA || useUbA) {
        InputIndex_constraints = ++numberOfInputs;
        ioData.input.emplace_back(InputIndex_constraints,
                                  std::vector<int>{Signal::DynamicSize, Signal::DynamicSize},
                                  DataType::DOUBLE);
    }
    if (useLbA) {
        InputIndex_lbA = ++numberOfInputs;
        ioData.input.emplace_back(
            InputIndex_lbA, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    }
    if (useUbA) {
        InputIndex_ubA = ++numberOfInputs;
        ioData.input.emplace_back(
            InputIndex_ubA, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    }
    if (useLb) {
        InputIndex_lb = ++numberOfInputs;
        ioData.input.emplace_back(
            InputIndex_lb, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    }
    if (useUb) {
        InputIndex_ub = ++numberOfInputs;
        ioData.input.emplace_back(
            InputIndex_ub, std::vector<int>{Signal::DynamicSize}, DataType::DOUBLE);
    }

    // Optional outputs
    int numberOfOutputs = OutputIndex::Status;
    if (computeObjVal) {
        OutputIndex_objVal = ++numberOfOutputs;
        ioData.output.emplace_back(OutputIndex_objVal, std::vector<int>{1}, DataType::DOUBLE);
    }

    if (!blockInfo->setIOPortsData(ioData)) {
        wbtError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool QpOases::initialize(BlockInformation* blockInfo)
{
    if (!Block::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!QpOases::parseParameters(blockInfo)) {
        wbtError << "Failed to parse parameters.";
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
        wbtError << "Failed to get parameters after their parsing.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    // Check the hessian size
    const auto size_H = blockInfo->getInputPortMatrixSize(InputIndex::Hessian);
    const auto numberOfVariables = size_H.first;
    if (size_H.first != size_H.second) {
        wbtError << "The Hessian matrix should be square.";
        return false;
    }

    // Check the gradient size
    const auto size_g = blockInfo->getInputPortWidth(InputIndex::Gradient);
    if (size_g != numberOfVariables) {
        wbtError << "The gradient size does not match with the Hessian size.";
        return false;
    }

    // Check the simple bounds size
    ok = true;
    if (pImpl->useLb) {
        ok = ok && (blockInfo->getInputPortWidth(InputIndex_lb) == numberOfVariables);
    }
    if (pImpl->useUb) {
        ok = ok && (blockInfo->getInputPortWidth(InputIndex_ub) == numberOfVariables);
    }
    if (!ok) {
        wbtError << "Sizes of bounds do not match with the number of variables.";
        return false;
    }

    int numberOfConstraints = 0;

    if (pImpl->useLbA || pImpl->useUbA) {
        // Check the constraints size
        const auto size_c = blockInfo->getInputPortMatrixSize(InputIndex_constraints);
        numberOfConstraints = size_c.first;
        if (size_c.second != numberOfVariables) {
            wbtError << "The column size of the constraints matrix does not match with "
                     << "the Hessian size";
            return false;
        }

        // Resize the buffer
        pImpl->constraints_rowMajor.resize(size_c.first, size_c.second);

        // Check the constraints' bound size
        bool ok = true;
        if (pImpl->useLbA) {
            ok = ok && (blockInfo->getInputPortWidth(InputIndex_lbA) == numberOfConstraints);
        }
        if (pImpl->useUbA) {
            ok = ok && (blockInfo->getInputPortWidth(InputIndex_ubA) == numberOfConstraints);
        }
        if (!ok) {
            wbtError << "Sizes of constraints' bounds do not match with the number of constraints.";
            return false;
        }
    }

    // Allocate the SQProblem.
    // SQProblem is used also without constraints. Other classes (e.g. QProblem, QProblemB) assume
    // fixed H and g and we cannot use them since we assume them as time varying quantities.
    pImpl->sqProblem = std::unique_ptr<qpOASES::SQProblem>(
        new qpOASES::SQProblem(numberOfVariables, numberOfConstraints));

    if (!pImpl->sqProblem) {
        wbtError << "Failed to allocate the QProblem or SQProblem object.";
        return false;
    }

    // Setup safe options
    qpOASES::Options problemOptions;
    problemOptions.setToDefault();
    pImpl->sqProblem->setOptions(problemOptions);

#ifdef NDEBUG
    pImpl->sqProblem->setPrintLevel(qpOASES::PL_NONE);
#else
    pImpl->sqProblem->setPrintLevel(qpOASES::PL_LOW);
#endif

    return true;
}

bool QpOases::initializeInitialConditions(const BlockInformation* /*blockInfo*/)
{
    pImpl->sqProblem->reset();
    return true;
}

bool QpOases::output(const BlockInformation* blockInfo)
{
    // INPUTS
    // ======

    // Get the Signals.
    // Note: the Hessian is symmetric, no need for conversion from column to row major.
    InputSignalPtr hessianSignal = blockInfo->getInputPortSignal(InputIndex::Hessian);
    InputSignalPtr gradientSignal = blockInfo->getInputPortSignal(InputIndex::Gradient);

    if (!hessianSignal || !gradientSignal) {
        wbtError << "Input signals not valid.";
        return false;
    }

    // OPTIONAL INPUTS
    // ===============

    const double* constraints = nullptr;
    const double* lbA = nullptr;
    const double* ubA = nullptr;
    const double* lb = nullptr;
    const double* ub = nullptr;

    if (pImpl->useLbA || pImpl->useUbA) {
        InputSignalPtr constraintsSignal = blockInfo->getInputPortSignal(InputIndex_constraints);
        if (!constraintsSignal) {
            wbtError << "Signal for lbA is not valid.";
            return false;
        }

        // Convert column-major buffers to row-major
        using namespace Eigen;
        using MatrixXdSimulink = Matrix<double, Dynamic, Dynamic, Eigen::ColMajor>;

        Map<MatrixXdSimulink> constraints_colMajor(
            const_cast<double*>(constraintsSignal->getBuffer<double>()),
            blockInfo->getInputPortMatrixSize(InputIndex_constraints).first,
            blockInfo->getInputPortMatrixSize(InputIndex_constraints).second);
        pImpl->constraints_rowMajor = constraints_colMajor;

        // Update the buffer passed to SQProblem
        constraints = pImpl->constraints_rowMajor.data();

        if (pImpl->useLbA) {
            InputSignalPtr lbASignal = blockInfo->getInputPortSignal(InputIndex_lbA);
            lbA = lbASignal->getBuffer<double>();
            if (!lbASignal) {
                wbtError << "Signal for lbA is not valid.";
                return false;
            }
        }

        if (pImpl->useUbA) {
            InputSignalPtr ubASignal = blockInfo->getInputPortSignal(InputIndex_ubA);
            ubA = ubASignal->getBuffer<double>();
            if (!ubASignal) {
                wbtError << "Signal for ubA is not valid.";
                return false;
            }
        }
    }

    if (pImpl->useLb) {
        InputSignalPtr lbSignal = blockInfo->getInputPortSignal(InputIndex_lb);
        lb = lbSignal->getBuffer<double>();
        if (!lbSignal) {
            wbtError << "Signal for lbA is not valid.";
            return false;
        }
    }

    if (pImpl->useUb) {
        InputSignalPtr ubSignal = blockInfo->getInputPortSignal(InputIndex_ub);
        ub = ubSignal->getBuffer<double>();
        if (!ubSignal) {
            wbtError << "Signal for ub is not valid.";
            return false;
        }
    }

    // OUTPUTS
    // =======

    OutputSignalPtr solutionSignal = blockInfo->getOutputPortSignal(OutputIndex::PrimalSolution);
    if (!solutionSignal) {
        wbtError << "Output signal not valid.";
        return false;
    }

    OutputSignalPtr statusSignal = blockInfo->getOutputPortSignal(OutputIndex::Status);
    if (!statusSignal) {
        wbtError << "Status signal not valid.";
        return false;
    }

    qpOASES::returnValue status;
    qpOASES::int_t nWSR = MaxIterations;

    if (pImpl->sqProblem->getCount() == 0) {
        // Initialize and solve first QP
        status = pImpl->sqProblem->init(hessianSignal->getBuffer<double>(),
                                        gradientSignal->getBuffer<double>(),
                                        constraints,
                                        lb,
                                        ub,
                                        lbA,
                                        ubA,
                                        nWSR,
                                        nullptr);
        if (pImpl->stopWhenFails && status != qpOASES::SUCCESSFUL_RETURN) {
            wbtError << "qpOASES: init() failed.";
            return false;
        }
    }
    else {
        // Solve the QP using hotstart technique
        status = pImpl->sqProblem->hotstart(hessianSignal->getBuffer<double>(),
                                            gradientSignal->getBuffer<double>(),
                                            constraints,
                                            lb,
                                            ub,
                                            lbA,
                                            ubA,
                                            nWSR,
                                            nullptr);

        // Handle possible errors
        if ((status != qpOASES::SUCCESSFUL_RETURN) && (status != qpOASES::RET_MAX_NWSR_REACHED)) {
            wbtWarning << "Internal qpOASES error. Trying to solve the problem with the remaining "
                       << "number of iterations.";
            pImpl->sqProblem->reset();

            nWSR = MaxIterations - nWSR;
            status = pImpl->sqProblem->init(hessianSignal->getBuffer<double>(),
                                            gradientSignal->getBuffer<double>(),
                                            constraints,
                                            lb,
                                            ub,
                                            lbA,
                                            ubA,
                                            nWSR,
                                            nullptr);
        }

        if (pImpl->stopWhenFails && status != qpOASES::SUCCESSFUL_RETURN) {
            wbtError << "qpOASES: hotstart() failed.";
            return false;
        }
    }

    const qpOASES::returnValue statusSol =
        pImpl->sqProblem->getPrimalSolution(solutionSignal->getBuffer<double>());

    if (pImpl->stopWhenFails && statusSol != qpOASES::SUCCESSFUL_RETURN) {
        wbtError << "qpOASES: getPrimalSolution() failed.";
        return false;
    }

    if (!statusSignal->set(0, qpOASES::getSimpleStatus(status))) {
        wbtError << "Failed to set status signal.";
        return false;
    }

    // OPTIONAL OUTPUTS
    // ================

    if (pImpl->computeObjVal) {
        const auto objVal = pImpl->sqProblem->getObjVal();

        OutputSignalPtr objValSignal = blockInfo->getOutputPortSignal(OutputIndex_objVal);
        if (!objValSignal) {
            wbtError << "Object Value signal not valid.";
            return false;
        }

        if (!objValSignal->set(0, objVal)) {
            wbtError << "Failed to set object value signal.";
            return false;
        }
    }

    return true;
}
