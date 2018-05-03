/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "CoderBlockInformation.h"
#include "Log.h"
#include "Parameter.h"
#include "Parameters.h"
#include "ToolboxSingleton.h"

#include <string>
#include <utility>
#include <vector>

using namespace wbt;

class CoderBlockInformation::impl
{
public:
    unsigned numberOfInputs;
    unsigned numberOfOutputs;

    std::vector<wbt::ParameterMetadata> paramsMetadata;
    std::unordered_map<BlockInformation::PortIndex, wbt::Signal> inputSignals;
    std::unordered_map<BlockInformation::PortIndex, wbt::Signal> outputSignals;

    std::string confBlockName;
    Parameters parametersFromRTW;

    std::unordered_map<BlockInformation::PortIndex, BlockInformation::MatrixSize> inputSignalSize;
    std::unordered_map<BlockInformation::PortIndex, BlockInformation::MatrixSize> outputSignalSize;
};

CoderBlockInformation::CoderBlockInformation()
    : pImpl{new CoderBlockInformation::impl()}
{}

CoderBlockInformation::~CoderBlockInformation() = default;

// BLOCK OPTIONS METHODS
// =====================

bool CoderBlockInformation::optionFromKey(const std::string& /*key*/, double& /*option*/) const
{
    return true;
}

// PORT INFORMATION SETTERS
// ========================

bool CoderBlockInformation::setNumberOfInputPorts(const unsigned& numberOfPorts)
{
    pImpl->numberOfInputs = numberOfPorts;
    return true;
}

bool CoderBlockInformation::setNumberOfOutputPorts(const unsigned& numberOfPorts)
{
    pImpl->numberOfInputs = numberOfPorts;
    return true;
}

bool CoderBlockInformation::setInputPortVectorSize(const PortIndex& idx, const VectorSize& size)
{
    pImpl->inputSignalSize[idx] = {1, size};
    return true;
}

bool CoderBlockInformation::setInputPortMatrixSize(const PortIndex& idx, const MatrixSize& size)
{
    pImpl->inputSignalSize[idx] = {size.first, size.second};
    return true;
}

bool CoderBlockInformation::setOutputPortVectorSize(const PortIndex& idx,
                                                    const VectorSize& portSize)
{
    pImpl->outputSignalSize[idx] = {1, portSize};
    return true;
}

bool CoderBlockInformation::setOutputPortMatrixSize(const PortIndex& idx, const MatrixSize& size)
{
    pImpl->outputSignalSize[idx] = {size.first, size.second};
    return true;
}

bool CoderBlockInformation::setInputPortType(const PortIndex& /*idx*/, const DataType& /*type*/)
{
    return true;
}

bool CoderBlockInformation::setOutputPortType(const PortIndex& /*idx*/, const DataType& /*type*/)
{
    return true;
}

// PORT INFORMATION GETTERS
// ========================

unsigned CoderBlockInformation::getInputPortWidth(const PortIndex& idx) const
{
    if (pImpl->inputSignalSize.find(idx) == pImpl->inputSignalSize.end()) {
        wbtError << "Failed to get width of signal at index " << idx << ".";
        return 0;
    }

    return static_cast<unsigned>(pImpl->inputSignalSize.at(idx).second);
}

unsigned CoderBlockInformation::getOutputPortWidth(const PortIndex& idx) const
{
    if (pImpl->outputSignalSize.find(idx) == pImpl->outputSignalSize.end()) {
        wbtError << "Failed to get width of signal at index " << idx << ".";
        return 0;
    }

    return static_cast<unsigned>(pImpl->outputSignalSize.at(idx).second);
}

wbt::Signal CoderBlockInformation::getInputPortSignal(const PortIndex& idx,
                                                      const VectorSize& size) const
{
    if (pImpl->inputSignals.find(idx) == pImpl->inputSignals.end()) {
        wbtError << "Trying to get non-existing signal " << idx << ".";
        return {};
    }

    // TODO: portWidth is used only if the signal is dynamically sized. In Simulink, in this case
    // the size is gathered from the SimStruct. From the coder instead? Is it possible having
    // a signal with dynamic size in the rtw file??
    // TODO: is it better this check or the one implemented in getOutputPortSignal?
    if (size != Signal::DynamicSize && pImpl->inputSignals.at(idx).getWidth() != size) {
        wbtError << "Signals with dynamic sizes (index " << idx
                 << ") are not supported by the CoderBlockInformation.";
        return {};
    }

    return pImpl->inputSignals.at(idx);
}

wbt::Signal CoderBlockInformation::getOutputPortSignal(const PortIndex& idx,
                                                       const VectorSize& /*size*/) const
{
    if (pImpl->outputSignals.find(idx) == pImpl->outputSignals.end()) {
        wbtError << "Trying to get non-existing signal " << idx << ".";
        return {};
    }

    if (pImpl->outputSignals.at(idx).getWidth() == Signal::DynamicSize) {
        wbtError << "Signals with dynamic sizes (index " << idx
                 << ") are not supported by the CoderBlockInformation.";
        return {};
    }

    return pImpl->outputSignals.at(idx);
}

BlockInformation::MatrixSize
CoderBlockInformation::getInputPortMatrixSize(const BlockInformation::PortIndex& idx) const
{
    if (pImpl->inputSignalSize.find(idx) == pImpl->inputSignalSize.end()) {
        wbtError << "Trying to get the size of non-existing signal " << idx << ".";
        return {};
    }

    return pImpl->inputSignalSize.at(idx);
}

BlockInformation::MatrixSize
CoderBlockInformation::getOutputPortMatrixSize(const BlockInformation::PortIndex& idx) const
{
    if (pImpl->outputSignalSize.find(idx) == pImpl->outputSignalSize.end()) {
        wbtError << "Trying to get the size of non-existing signal " << idx << ".";
        return {};
    }

    return pImpl->outputSignalSize.at(idx);
}

bool CoderBlockInformation::addParameterMetadata(const wbt::ParameterMetadata& paramMD)
{
    for (auto md : pImpl->paramsMetadata) {
        if (md.name == paramMD.name) {
            wbtError << "Trying to store an already existing " << md.name << " parameter.";
            return false;
        }
    }

    pImpl->paramsMetadata.push_back(paramMD);
    return true;
}

// PARAMETERS METHODS
// ==================

bool CoderBlockInformation::parseParameters(wbt::Parameters& parameters)
{
    if (pImpl->parametersFromRTW.getNumberOfParameters() == 0) {
        wbtError << "The Parameters object containing the parameters to parse is empty.";
        return false;
    }

    for (wbt::ParameterMetadata md : pImpl->paramsMetadata) {
        // Check that all the parameters that are parsed have already been stored from the coder
        if (!pImpl->parametersFromRTW.existName(md.name)) {
            wbtError << "Trying to get a parameter value for " << md.name
                     << ", but its value has never been stored.";
            return false;
        }
        // Check if the parameters are not dynamically sized
        if (!(md.rows == -1 || md.cols == -1)
            && md != pImpl->parametersFromRTW.getParameterMetadata(md.name)) {
            wbtError << "Dynamically sized parameters are not supported.";
            return false;
        }
    }

    // This implementation of BlockInformation contains all the parameters from the very beginning,
    // stored using the storeRTWParameters method. Here for simplicity all the stored parameters are
    // returned, even if the metadata contain only a subset of them.
    parameters = pImpl->parametersFromRTW;
    return true;
}

bool CoderBlockInformation::storeRTWParameters(const Parameters& parameters)
{
    if (parameters.getNumberOfParameters() == 0) {
        wbtError << "The Parameters object passed doesn't contain any parameter.";
        return false;
    }

    pImpl->parametersFromRTW = parameters;

    // This code is shared with SimulinkBlockInformation::parseParameters
    //
    // Check if the parameters object contains all the information for creating a
    // Configuration object.
    if (Parameters::containConfigurationData(parameters)) {
        if (!ToolboxSingleton::sharedInstance().storeConfiguration(parameters)) {
            wbtError << "Failed to store a Configuration object in the ToolboxSigleton.";
            return false;
        }
        // Save the name of the Configuration block which the processed WBBlock refers to
        if (!parameters.getParameter("ConfBlockName", pImpl->confBlockName)) {
            wbtError << "Failed to read ConfBlockName parameter from the Parameters object "
                     << "that should store Configuration data.";
            return false;
        }
    }

    return true;
}

bool CoderBlockInformation::setInputSignal(const PortIndex& portNumber,
                                           void* address,
                                           const MatrixSize& portSize)
{
    if (pImpl->inputSignals.find(portNumber) != pImpl->inputSignals.end()) {
        wbtError << "The signal " << portNumber << "has already been previously stored.";
        return false;
    }

    if (!address) {
        wbtError << "The pointer to the signal to store is a nullptr.";
        return false;
    }

    bool isConst = true;
    pImpl->inputSignals.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(portNumber),
        std::forward_as_tuple(Signal::DataFormat::CONTIGUOUS_ZEROCOPY, DataType::DOUBLE, isConst));

    pImpl->inputSignals[portNumber].setWidth(portSize.first * portSize.second);
    return pImpl->inputSignals[portNumber].initializeBufferFromContiguousZeroCopy(address);
}

bool CoderBlockInformation::setOutputSignal(const PortIndex& portNumber,
                                            void* address,
                                            const MatrixSize& portSize)
{
    if (pImpl->outputSignals.find(portNumber) != pImpl->outputSignals.end()) {
        wbtError << "The signal " << portNumber << "has already been previously stored.";
        return false;
    }

    if (!address) {
        wbtError << "The pointer to the signal to store is a nullptr.";
        return false;
    }

    bool isConst = false;
    pImpl->outputSignals.emplace(
        std::piecewise_construct,
        std::forward_as_tuple(portNumber),
        std::forward_as_tuple(Signal::DataFormat::CONTIGUOUS_ZEROCOPY, DataType::DOUBLE, isConst));

    pImpl->outputSignals[portNumber].setWidth(portSize.first * portSize.second);
    return pImpl->outputSignals[portNumber].initializeBufferFromContiguousZeroCopy(address);
}

std::weak_ptr<wbt::RobotInterface> CoderBlockInformation::getRobotInterface() const
{
    if (pImpl->confBlockName.empty()) {
        wbtError << "No ConfBlockName stored. Failed to get RobotInterface object.";
        // Return an empty weak pointer
        return std::weak_ptr<wbt::RobotInterface>();
    }

    ToolboxSingleton& interface = ToolboxSingleton::sharedInstance();
    return interface.getRobotInterface(pImpl->confBlockName);
}

std::weak_ptr<iDynTree::KinDynComputations> CoderBlockInformation::getKinDynComputations() const
{
    if (pImpl->confBlockName.empty()) {
        wbtError << "No ConfBlockName stored. Failed to get KinDynComputations object.";
        // Return an empty weak pointer
        return std::weak_ptr<iDynTree::KinDynComputations>();
    }

    ToolboxSingleton& interface = ToolboxSingleton::sharedInstance();
    return interface.getKinDynComputations(pImpl->confBlockName);
}
