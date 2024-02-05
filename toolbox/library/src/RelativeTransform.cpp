/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "WBToolbox/Block/RelativeTransform.h"
#include "WBToolbox/Base/Configuration.h"
#include "WBToolbox/Base/RobotInterface.h"

#include <BlockFactory/Core/BlockInformation.h>
#include <BlockFactory/Core/Log.h>
#include <BlockFactory/Core/Parameter.h>
#include <BlockFactory/Core/Parameters.h>
#include <BlockFactory/Core/Signal.h>
#include <Eigen/Core>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Transform.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Indices.h>

#include <ostream>

using namespace wbt::block;
using namespace blockfactory::core;

// INDICES: PARAMETERS, INPUTS, OUTPUT
// ===================================

enum ParamIndex
{
    Bias = wbt::base::WBBlock::NumberOfParameters - 1,
    Frame1,
    Frame2
};

enum InputIndex
{
    JointConfiguration = 0,
};

enum OutputIndex
{
    Transform = 0,
};

// BLOCK PIMPL
// ===========

class RelativeTransform::impl
{
public:
    iDynTree::VectorDynSize jointConfiguration;
    iDynTree::FrameIndex frame1Index = iDynTree::FRAME_INVALID_INDEX;
    iDynTree::FrameIndex frame2Index = iDynTree::FRAME_INVALID_INDEX;
};

// BLOCK CLASS
// ===========

RelativeTransform::RelativeTransform()
    : pImpl{new impl()}
{}

RelativeTransform::~RelativeTransform() = default;

unsigned RelativeTransform::numberOfParameters()
{
    return WBBlock::numberOfParameters() + 2;
}

bool RelativeTransform::parseParameters(BlockInformation* blockInfo)
{
    const std::vector<ParameterMetadata> metadata{
        {ParameterType::STRING, ParamIndex::Frame1, 1, 1, "Frame1"},
        {ParameterType::STRING, ParamIndex::Frame2, 1, 1, "Frame2"}};

    for (const auto& md : metadata) {
        if (!blockInfo->addParameterMetadata(md)) {
            bfError << "Failed to store parameter metadata";
            return false;
        }
    }

    return blockInfo->parseParameters(m_parameters);
}

bool RelativeTransform::configureSizeAndPorts(BlockInformation* blockInfo)
{
    if (!WBBlock::configureSizeAndPorts(blockInfo)) {
        return false;
    }

    // Get the DoFs
    const int dofs = getRobotInterface()->getConfiguration().getNumberOfDoFs();

    // INPUTS
    // ======
    //
    // 1) Joints position (1xDoFs vector)
    //
    // OUTPUTS
    // =======
    //
    // 1) Homogeneous transformation between frame1 and frame2 (4x4 matrix)
    //

    const bool ok = blockInfo->setPortsInfo(
        {
            // Inputs
            {InputIndex::JointConfiguration, Port::Dimensions{dofs}, Port::DataType::DOUBLE},
        },
        {
            // Outputs
            {OutputIndex::Transform, Port::Dimensions{4, 4}, Port::DataType::DOUBLE},

        });

    if (!ok) {
        bfError << "Failed to configure input / output ports.";
        return false;
    }

    return true;
}

bool RelativeTransform::initialize(BlockInformation* blockInfo)
{
    if (!WBBlock::initialize(blockInfo)) {
        return false;
    }

    // PARAMETERS
    // ==========

    if (!parseParameters(blockInfo)) {
        bfError << "Failed to parse parameters.";
        return false;
    }

    std::string frame1;
    std::string frame2;

    bool ok = true;
    ok = ok && m_parameters.getParameter("Frame1", frame1);
    ok = ok && m_parameters.getParameter("Frame2", frame2);

    if (!ok) {
        bfError << "Failed to get parameters after their parsing.";
        return false;
    }

    // CLASS INITIALIZATION
    // ====================

    // Check if the frames are valid
    // -----------------------------

    const auto kinDyn = getKinDynComputations();
    if (!kinDyn) {
        bfError << "Cannot retrieve handle to KinDynComputations.";
        return false;
    }

    // Frame 1
    pImpl->frame1Index = kinDyn->getFrameIndex(frame1);
    if (pImpl->frame1Index == iDynTree::FRAME_INVALID_INDEX) {
        bfError << "Cannot find " + frame1 + " in the frame list.";
        return false;
    }

    // Frame 2
    pImpl->frame2Index = kinDyn->getFrameIndex(frame2);
    if (pImpl->frame2Index == iDynTree::FRAME_INVALID_INDEX) {
        bfError << "Cannot find " + frame2 + " in the frame list.";
        return false;
    }

    // Initialize the buffer
    pImpl->jointConfiguration.resize(kinDyn->getNrOfDegreesOfFreedom());
    pImpl->jointConfiguration.zero();

    return true;
}

bool RelativeTransform::terminate(const BlockInformation* blockInfo)
{
    return WBBlock::terminate(blockInfo);
}

bool RelativeTransform::output(const BlockInformation* blockInfo)
{
    using namespace Eigen;
    using Matrix4dSimulink = Matrix<double, 4, 4, Eigen::ColMajor>;
    using Matrix4diDynTree = Matrix<double, 4, 4, Eigen::RowMajor>;

    // Get the KinDynComputations object
    auto kinDyn = getKinDynComputations();
    if (!kinDyn) {
        bfError << "Failed to retrieve the KinDynComputations object.";
        return false;
    }

    // GET THE SIGNALS
    // ===============

    InputSignalPtr jointsPosSig = blockInfo->getInputPortSignal(InputIndex::JointConfiguration);

    if (!jointsPosSig) {
        bfError << "Input signals not valid.";
        return false;
    }

    for (unsigned i = 0; i < jointsPosSig->getWidth(); ++i) {
        pImpl->jointConfiguration.setVal(i, jointsPosSig->get<double>(i));
    }

    kinDyn->setJointPos(pImpl->jointConfiguration);

    // OUTPUT
    // ======

    iDynTree::Transform frame1_H_frame2;

    // Compute the relative transform
    frame1_H_frame2 = kinDyn->getRelativeTransform(pImpl->frame1Index, pImpl->frame2Index);
    std::cerr << "RelativeTransform: frame1_H_frame2: " << frame1_H_frame2.toString() << std::endl;

    // Get the output signal memory location
    OutputSignalPtr output = blockInfo->getOutputPortSignal(OutputIndex::Transform);
    if (!output) {
        bfError << "Output signal not valid.";
        return false;
    }

    // Allocate objects for row-major -> col-major conversion
    Map<const Matrix4diDynTree> frame1_H_frame2_RowMajor =
        toEigen(frame1_H_frame2.asHomogeneousTransform());
    Map<Matrix4dSimulink> frame1_H_frame2_ColMajor(output->getBuffer<double>(), 4, 4);

    // Forward the buffer to Simulink transforming it to ColMajor
    frame1_H_frame2_ColMajor = frame1_H_frame2_RowMajor;
    return true;
}
