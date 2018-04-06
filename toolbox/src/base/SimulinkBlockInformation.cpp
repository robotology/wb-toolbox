/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "SimulinkBlockInformation.h"
#include "Log.h"
#include "MxAnyType.h"
#include "Parameters.h"
#include "Signal.h"
#include "ToolboxSingleton.h"

#include <algorithm>
#include <string>
#include <vector>

using namespace wbt;

SimulinkBlockInformation::SimulinkBlockInformation(SimStruct* S)
    : simstruct(S)
{}

// BLOCK OPTIONS METHODS
// =====================

bool SimulinkBlockInformation::optionFromKey(const std::string& key, double& option) const
{
    if (key == wbt::BlockOptionPrioritizeOrder) {
        option = SS_OPTION_PLACE_ASAP;
        return true;
    }

    wbtError << "Unrecognized block option.";
    return false;
}

// PARAMETERS METHODS
// ==================

bool SimulinkBlockInformation::getStringParameterAtIndex(const ParameterIndex& idx,
                                                         std::string& value) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, idx);
    return MxAnyType(blockParam).asString(value);
}

bool SimulinkBlockInformation::getScalarParameterAtIndex(const ParameterIndex& idx,
                                                         double& value) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, idx);
    return MxAnyType(blockParam).asDouble(value);
}

bool SimulinkBlockInformation::getBooleanParameterAtIndex(const ParameterIndex& idx,
                                                          bool& value) const
{
    double tmpValue = 0;
    const mxArray* blockParam = ssGetSFcnParam(simstruct, idx);

    // The Simulink mask often doesn't store boolean data from the mask as bool but as double.
    // Calling asBool() will fail in this case. If this happens, asDouble() is used as fallback.
    if (MxAnyType(blockParam).asBool(value)) {
        return true;
    }
    else if (MxAnyType(blockParam).asDouble(tmpValue)) {
        value = static_cast<bool>(tmpValue);
        return true;
    }

    wbtError << "Failed to parse bool parameter";
    return false;
}

bool SimulinkBlockInformation::getCellAtIndex(const ParameterIndex& idx, AnyCell& value) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, idx);
    return MxAnyType(blockParam).asAnyCell(value);
}

bool SimulinkBlockInformation::getStructAtIndex(const ParameterIndex& idx, AnyStruct& value) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, idx);
    return MxAnyType(blockParam).asAnyStruct(value);
}

bool SimulinkBlockInformation::getVectorAtIndex(const ParameterIndex& idx,
                                                std::vector<double>& value) const
{
    const mxArray* blockParam = ssGetSFcnParam(simstruct, idx);
    return MxAnyType(blockParam).asVectorDouble(value);
}

bool SimulinkBlockInformation::getStringFieldAtIndex(const ParameterIndex& idx,
                                                     const std::string& fieldName,
                                                     std::string& value) const
{
    AnyStruct s;

    if (!getStructAtIndex(idx, s)) {
        wbtError << "Failed to get struct at index " << idx << ".";
        return false;
    }

    if (s.find(fieldName) == s.end()) {
        wbtError << "Struct at index " << idx << " does not contain any " << fieldName << " field.";
        return false;
    }

    return s.at(fieldName)->asString(value);
}

bool SimulinkBlockInformation::getScalarFieldAtIndex(const ParameterIndex& idx,
                                                     const std::string& fieldName,
                                                     double& value) const
{
    AnyStruct s;

    if (!getStructAtIndex(idx, s)) {
        wbtError << "Failed to get struct at index " << idx << ".";
        return false;
    }

    if (s.find(fieldName) == s.end()) {
        wbtError << "Struct at index " << idx << " does not contain any " << fieldName << " field.";
        return false;
    }

    return s.at(fieldName)->asDouble(value);
}

bool SimulinkBlockInformation::getBooleanFieldAtIndex(const ParameterIndex& idx,
                                                      const std::string& fieldName,
                                                      bool& value) const
{
    AnyStruct s;

    if (!getStructAtIndex(idx, s)) {
        wbtError << "Failed to get struct at index " << idx << ".";
        return false;
    }

    if (s.find(fieldName) == s.end()) {
        wbtError << "Struct at index " << idx << " does not contain any " << fieldName << " field.";
        return false;
    }

    return s.at(fieldName)->asBool(value);
}

bool SimulinkBlockInformation::getCellFieldAtIndex(const ParameterIndex& idx,
                                                   const std::string& fieldName,
                                                   AnyCell& value) const
{
    AnyStruct s;

    if (!getStructAtIndex(idx, s)) {
        wbtError << "Failed to get struct at index " << idx << ".";
        return false;
    }

    if (s.find(fieldName) == s.end()) {
        wbtError << "Struct at index " << idx << " does not contain any " << fieldName << " field.";
        return false;
    }

    return s.at(fieldName)->asAnyCell(value);
}

bool SimulinkBlockInformation::getVectorDoubleFieldAtIndex(const ParameterIndex& idx,
                                                           const std::string& fieldName,
                                                           std::vector<double>& value) const
{
    AnyStruct s;

    if (!getStructAtIndex(idx, s)) {
        wbtError << "Failed to get struct at index " << idx << ".";
        return false;
    }

    if (s.find(fieldName) == s.end()) {
        wbtError << "Struct at index " << idx << " does not contain any " << fieldName << " field.";
        return false;
    }

    return s.at(fieldName)->asVectorDouble(value);
}

// PORT INFORMATION SETTERS
// ========================

bool SimulinkBlockInformation::setNumberOfInputPorts(const unsigned& numberOfPorts)
{
    return ssSetNumInputPorts(simstruct, numberOfPorts);
}

bool SimulinkBlockInformation::setNumberOfOutputPorts(const unsigned& numberOfPorts)
{
    return ssSetNumOutputPorts(simstruct, numberOfPorts);
}

bool SimulinkBlockInformation::setInputPortVectorSize(const SignalIndex& idx,
                                                      const VectorSize& size)
{
    if (size == Signal::DynamicSize) {
        // TODO: in this case, explore how to use mdlSetOutputPortDimensionInfo and
        // mdlSetDefaultPortDimensionInfo
        return ssSetInputPortVectorDimension(simstruct, idx, DYNAMICALLY_SIZED);
    }

    return ssSetInputPortVectorDimension(simstruct, idx, size);
}

bool SimulinkBlockInformation::setInputPortMatrixSize(const SignalIndex& idx,
                                                      const MatrixSize& size)
{
    // Refer to: https://it.mathworks.com/help/simulink/sfg/sssetoutputportmatrixdimensions.html
    if (size.first == Signal::DynamicSize || size.second == Signal::DynamicSize) {
        // TODO: in this case, explore how to use mdlSetOutputPortDimensionInfo and
        // mdlSetDefaultPortDimensionInfo
        ssSetInputPortMatrixDimensions(simstruct, idx, DYNAMICALLY_SIZED, DYNAMICALLY_SIZED);
    }

    return ssSetInputPortMatrixDimensions(simstruct, idx, size.first, size.first);
}

bool SimulinkBlockInformation::setOutputPortVectorSize(const SignalIndex& idx,
                                                       const VectorSize& size)
{
    if (size == Signal::DynamicSize) {
        // TODO: in this case, explore how to use mdlSetOutputPortDimensionInfo and
        // mdlSetDefaultPortDimensionInfo
        return ssSetOutputPortVectorDimension(simstruct, idx, DYNAMICALLY_SIZED);
    }

    return ssSetOutputPortVectorDimension(simstruct, idx, size);
}

bool SimulinkBlockInformation::setOutputPortMatrixSize(const SignalIndex& idx,
                                                       const MatrixSize& size)
{
    // Refer to: https://it.mathworks.com/help/simulink/sfg/sssetinputportmatrixdimensions.html
    if (size.first == Signal::DynamicSize || size.second == Signal::DynamicSize) {
        // TODO: in this case, explore how to use mdlSetOutputPortDimensionInfo and
        // mdlSetDefaultPortDimensionInfo
        return ssSetOutputPortMatrixDimensions(
            simstruct, idx, DYNAMICALLY_SIZED, DYNAMICALLY_SIZED);
    }

    return ssSetOutputPortMatrixDimensions(simstruct, idx, size.first, size.second);
}

bool SimulinkBlockInformation::setInputPortType(const SignalIndex& idx, const DataType& type)
{
    ssSetInputPortDirectFeedThrough(simstruct, idx, 1);
    ssSetInputPortDataType(simstruct, idx, mapPortTypeToSimulink(type));
    return true;
}

bool SimulinkBlockInformation::setOutputPortType(const SignalIndex& idx, const DataType& type)
{
    ssSetOutputPortDataType(simstruct, idx, mapPortTypeToSimulink(type));
    return true;
}

// PORT INFORMATION GETTERS
// ========================

unsigned SimulinkBlockInformation::getInputPortWidth(const SignalIndex& idx) const
{
    return ssGetInputPortWidth(simstruct, idx);
}

unsigned SimulinkBlockInformation::getOutputPortWidth(const SignalIndex& idx) const
{
    return ssGetOutputPortWidth(simstruct, idx);
}

Signal SimulinkBlockInformation::getInputPortSignal(const SignalIndex& idx,
                                                    const VectorSize& size) const
{
    // Read if the signal is contiguous or non-contiguous
    boolean_T isContiguous = ssGetInputPortRequiredContiguous(simstruct, idx);
    Signal::DataFormat sigDataFormat =
        isContiguous ? Signal::DataFormat::CONTIGUOUS_ZEROCOPY : Signal::DataFormat::NONCONTIGUOUS;

    // Check if the signal is dynamically sized (which means that the dimension
    // cannot be read)
    bool isDynamicallySized = (ssGetInputPortWidth(simstruct, idx) == DYNAMICALLY_SIZED);

    // Note that if the signal is dynamically sized, portWidth is necessary
    if (isDynamicallySized && size == Signal::DynamicSize) {
        wbtError << "Trying to get a dynamically sized signal without specifying its size.";
        return {};
    }

    // Read the width of the signal if it is not provided as input and the signal is not
    // dynamically sized
    VectorSize signalSize = size;
    if (!isDynamicallySized && size == Signal::DynamicSize) {
        signalSize = ssGetInputPortWidth(simstruct, idx);
    }

    // Get the data type of the Signal if set (default: double)
    DTypeId dataType = ssGetInputPortDataType(simstruct, idx);

    switch (sigDataFormat) {
        case Signal::DataFormat::CONTIGUOUS_ZEROCOPY: {
            // Initialize the signal
            bool isConstPort = true;
            Signal signal(Signal::DataFormat::CONTIGUOUS_ZEROCOPY,
                          mapSimulinkToPortType(dataType),
                          isConstPort);
            signal.setWidth(signalSize);
            // Initialize signal's data
            if (!signal.initializeBufferFromContiguousZeroCopy(
                    ssGetInputPortSignal(simstruct, idx))) {
                wbtError << "Failed to inititialize CONTIGUOUS_ZEROCOPY signal at index " << idx
                         << ".";
                return {};
            }
            return signal;
        }
        case Signal::DataFormat::NONCONTIGUOUS: {
            // Initialize the signal
            bool isConstPort = true;
            Signal signal(
                Signal::DataFormat::NONCONTIGUOUS, mapSimulinkToPortType(dataType), isConstPort);
            signal.setWidth(signalSize);
            // Initialize signal's data
            InputPtrsType port = ssGetInputPortSignalPtrs(simstruct, idx);
            if (!signal.initializeBufferFromNonContiguous(static_cast<const void* const*>(port))) {
                wbtError << "Failed to inititialize NONCONTIGUOUS signal at index " << idx << ".";
                return {};
            }
            return signal;
        }
        case Signal::DataFormat::CONTIGUOUS: {
            wbtError << "Failed to inititialize CONTIGUOUS signal at index " << idx << "."
                     << std::endl
                     << "CONTIGUOUS input signals are not yet supported."
                     << "Use CONTIGUOUS_ZEROCOPY instead.";
            return {};
        }
    }
}

wbt::Signal SimulinkBlockInformation::getOutputPortSignal(const SignalIndex& idx,
                                                          const VectorSize& size) const
{
    // Check if the signal is dynamically sized (which means that the dimension
    // cannot be read)
    bool isDynamicallySized = (ssGetOutputPortWidth(simstruct, idx) == DYNAMICALLY_SIZED);

    // Note that if the signal is dynamically sized, portWidth is necessary
    if (isDynamicallySized && size == Signal::DynamicSize) {
        wbtError << "Trying to get a dynamically sized signal without specifying its size.";
        return {};
    }

    // Read the width of the signal if it is not provided as input and the signal is not
    // dynamically sized
    VectorSize signalSize = size;
    if (!isDynamicallySized && size == Signal::DynamicSize) {
        signalSize = ssGetOutputPortWidth(simstruct, idx);
    }

    // Get the data type of the Signal if set (default: double)
    DTypeId dataType = ssGetOutputPortDataType(simstruct, idx);

    bool isConstPort = false;
    Signal signal(
        Signal::DataFormat::CONTIGUOUS_ZEROCOPY, mapSimulinkToPortType(dataType), isConstPort);
    signal.setWidth(signalSize);

    if (!signal.initializeBufferFromContiguousZeroCopy(ssGetOutputPortSignal(simstruct, idx))) {
        wbtError << "Failed to inititialize CONTIGUOUS_ZEROCOPY signal at index " << idx << ".";
        return {};
    }

    return signal;
}

BlockInformation::MatrixSize
SimulinkBlockInformation::getInputPortMatrixSize(const SignalIndex& idx) const
{
    if (ssGetInputPortNumDimensions(simstruct, idx) < 2) {
        wbtError << "Signal at index " << idx
                 << "does not contain a matrix. Failed to gete its size.";
        return {};
    }

    const int_T* sizes = ssGetInputPortDimensions(simstruct, idx);
    return {sizes[0], sizes[1]};
}

BlockInformation::MatrixSize
SimulinkBlockInformation::getOutputPortMatrixSize(const SignalIndex& idx) const
{
    if (ssGetOutputPortNumDimensions(simstruct, idx) < 2) {
        wbtError << "Signal at index " << idx
                 << "does not contain a matrix. Failed to gete its size.";
        return {};
    }

    const int_T* sizes = ssGetOutputPortDimensions(simstruct, idx);
    return {sizes[0], sizes[1]};
}

DataType SimulinkBlockInformation::mapSimulinkToPortType(const DTypeId& typeId) const
{
    switch (typeId) {
        case SS_DOUBLE:
            return DataType::DOUBLE;
        case SS_SINGLE:
            return DataType::SINGLE;
        case SS_INT8:
            return DataType::INT8;
        case SS_UINT8:
            return DataType::UINT8;
        case SS_INT16:
            return DataType::INT16;
        case SS_UINT16:
            return DataType::UINT16;
        case SS_INT32:
            return DataType::INT32;
        case SS_UINT32:
            return DataType::UINT32;
        case SS_BOOLEAN:
            return DataType::BOOLEAN;
        default:
            return DataType::DOUBLE;
    }
}

DTypeId SimulinkBlockInformation::mapPortTypeToSimulink(const DataType& dataType) const
{
    switch (dataType) {
        case DataType::DOUBLE:
            return SS_DOUBLE;
        case DataType::SINGLE:
            return SS_SINGLE;
        case DataType::INT8:
            return SS_INT8;
        case DataType::UINT8:
            return SS_UINT8;
        case DataType::INT16:
            return SS_INT16;
        case DataType::UINT16:
            return SS_UINT16;
        case DataType::INT32:
            return SS_INT32;
        case DataType::UINT32:
            return SS_UINT32;
        case DataType::BOOLEAN:
            return SS_BOOLEAN;
    }
}

bool SimulinkBlockInformation::addParameterMetadata(const wbt::ParameterMetadata& paramMD)
{
    for (auto md : m_paramsMetadata) {
        if (md.m_name == paramMD.m_name) {
            wbtError << "Trying to store an already existing " << md.m_name << " parameter.";
            return false;
        }
    }

    // Add the new metadata to the block information
    m_paramsMetadata.push_back(paramMD);
    return true;
}

bool SimulinkBlockInformation::parseParameters(wbt::Parameters& parameters)
{
    auto metadataContainsScalarParam = [](const wbt::ParameterMetadata& md) -> const bool {
        return md.m_rows == 1 && md.m_cols == 1;
    };

    for (wbt::ParameterMetadata paramMD : m_paramsMetadata) {

        bool ok;

        // TODO Right now the cells are reshaped to a 1 x NumElements by MxAnyType
        if (paramMD.m_rows == ParameterMetadata::DynamicSize) {
            wbtError << "Dynamically sized rows are not currently supported.";
            return false;
        }

        // Handle the case of dynamically sized columns. In this case the metadata passed
        // from the Block (containing DynamicSize) is modified with the length of the
        // vector that is going to be stored.
        const bool hasDynSizeColumns = (paramMD.m_cols == ParameterMetadata::DynamicSize);

        switch (paramMD.m_type) {
            // SCALAR / VECTOR PARAMETERS
            // --------------------------
            //
            // getScalarParameterAtIndex and getVectorAtIndex operate on type double.
            // The cast to other types is handled by storeParameter internally,
            // accordingly to the type stored in the metadata.
            //
            // Despite bool has its own bool parser, considering that both int and double
            // are loaded as double (Simulink limitation), in order to simplify the
            // maintainability of this code, everything is handled as double.
            //
            case PARAM_INT:
            case PARAM_BOOL:
            case PARAM_DOUBLE: {
                if (metadataContainsScalarParam(paramMD)) {
                    double paramValue;
                    if (!getScalarParameterAtIndex(paramMD.m_index, paramValue)) {
                        wbtError << "Failed to get scalar parameter at index " << paramMD.m_index
                                 << ".";
                        return false;
                    }
                    ok = parameters.storeParameter<double>(paramValue, paramMD);
                }
                else {
                    std::vector<double> paramVector;
                    if (!getVectorAtIndex(paramMD.m_index, paramVector)) {
                        wbtError << "Failed to get vector parameter at index " << paramMD.m_index
                                 << ".";
                        return false;
                    }
                    if (hasDynSizeColumns) {
                        paramMD.m_cols = paramVector.size();
                    }
                    ok = parameters.storeParameter<double>(paramVector, paramMD);
                }
                break;
            }
            case PARAM_STRING: {
                if (metadataContainsScalarParam(paramMD)) {
                    std::string paramValue;
                    if (!getStringParameterAtIndex(paramMD.m_index, paramValue)) {
                        wbtError << "Failed to get string parameter at index " << paramMD.m_index
                                 << ".";
                        return false;
                    }
                    ok = parameters.storeParameter<std::string>(paramValue, paramMD);
                }
                else {
                    wbtError << "Char arrays are not yet supported.";
                    return false;
                }
                break;
            }
            // CELL PARAMETERS
            // ---------------
            case PARAM_CELL_INT:
            case PARAM_CELL_BOOL:
            case PARAM_CELL_DOUBLE: {
                AnyCell cell;
                if (!getCellAtIndex(paramMD.m_index, cell)) {
                    wbtError << "Failed to get cell parameter at index " << paramMD.m_index << ".";
                    return false;
                }
                std::vector<double> paramVector;
                for (auto element : cell) {
                    double value;
                    if (!element->asDouble(value)) {
                        wbtError << "Failed to parse an element of the cell at index "
                                 << paramMD.m_index << " as a double.";
                        return false;
                    }
                    paramVector.push_back(value);
                }
                if (hasDynSizeColumns) {
                    paramMD.m_cols = paramVector.size();
                }
                ok = parameters.storeParameter<double>(paramVector, paramMD);
                break;
            }
            case PARAM_CELL_STRING: {
                AnyCell cell;
                if (!getCellAtIndex(paramMD.m_index, cell)) {
                    wbtError << "Failed to get cell parameter at index " << paramMD.m_index << ".";
                    return false;
                }
                std::vector<std::string> paramVector;
                for (auto element : cell) {
                    std::string value;
                    if (!element->asString(value)) {
                        wbtError << "Failed to parse an element of the cell at index "
                                 << paramMD.m_index << " as a string.";
                        return false;
                    }
                    paramVector.push_back(value);
                }
                if (hasDynSizeColumns) {
                    paramMD.m_cols = paramVector.size();
                }
                ok = parameters.storeParameter<std::string>(paramVector, paramMD);
                break;
            }
            // STRUCT PARAMETERS
            // -----------------
            case PARAM_STRUCT_INT:
            case PARAM_STRUCT_BOOL:
            case PARAM_STRUCT_DOUBLE: {
                if (metadataContainsScalarParam(paramMD)) {
                    double paramValue;
                    if (!getScalarFieldAtIndex(paramMD.m_index, paramMD.m_name, paramValue)) {
                        wbtError << "Failed to get struct parameter at index " << paramMD.m_index
                                 << ".";
                        return false;
                    }
                    ok = parameters.storeParameter<double>(paramValue, paramMD);
                }
                else {
                    std::vector<double> paramVector;
                    if (!getVectorDoubleFieldAtIndex(
                            paramMD.m_index, paramMD.m_name, paramVector)) {
                        wbtError << "Failed to get vector field " << paramMD.m_name
                                 << "from the struct at index " << paramMD.m_index << ".";
                        return false;
                    }
                    if (hasDynSizeColumns) {
                        paramMD.m_cols = paramVector.size();
                    }
                    ok = parameters.storeParameter<double>(paramVector, paramMD);
                }
                break;
            }
            case PARAM_STRUCT_STRING: {
                if (metadataContainsScalarParam(paramMD)) {
                    std::string paramValue;
                    if (!getStringFieldAtIndex(paramMD.m_index, paramMD.m_name, paramValue)) {
                        wbtError << "Failed to get string field " << paramMD.m_name
                                 << "from the struct at index " << paramMD.m_index << ".";
                        return false;
                    }
                    ok = parameters.storeParameter<std::string>(paramValue, paramMD);
                }
                else {
                    wbtError << "Char arrays are not yet supported.";
                    return false;
                }
                break;
            }
            case PARAM_STRUCT_CELL_INT:
            case PARAM_STRUCT_CELL_BOOL:
            case PARAM_STRUCT_CELL_DOUBLE: {
                AnyCell cell;
                std::vector<double> paramVector;
                if (!getCellFieldAtIndex(paramMD.m_index, paramMD.m_name, cell)) {
                    wbtError << "Failed to get cell field " << paramMD.m_name
                             << " from the struct at index " << paramMD.m_index << ".";
                    return false;
                }
                for (auto element : cell) {
                    double value;
                    if (!element->asDouble(value)) {
                        wbtError << "Failed to parse an element of the cell field "
                                 << paramMD.m_name << " from the struct at index "
                                 << paramMD.m_index << " as a double.";
                        return false;
                    }
                    paramVector.push_back(value);
                }
                if (hasDynSizeColumns) {
                    paramMD.m_cols = paramVector.size();
                }
                ok = parameters.storeParameter<double>(paramVector, paramMD);
                break;
            }
            case PARAM_STRUCT_CELL_STRING: {
                AnyCell cell;
                std::vector<std::string> paramVector;
                if (!getCellFieldAtIndex(paramMD.m_index, paramMD.m_name, cell)) {
                    wbtError << "Failed to get cell field " << paramMD.m_name
                             << " from the struct at index " << paramMD.m_index << ".";
                    return false;
                }
                for (auto element : cell) {
                    std::string value;
                    if (!element->asString(value)) {
                        wbtError << "Failed to parse an element of the cell field "
                                 << paramMD.m_name << " from the struct at index "
                                 << paramMD.m_index << " as a string.";
                        return false;
                    }
                    paramVector.push_back(value);
                }
                if (hasDynSizeColumns) {
                    paramMD.m_cols = paramVector.size();
                }
                ok = parameters.storeParameter<std::string>(paramVector, paramMD);
                break;
            }
        }

        if (!ok) {
            wbtError << "Failed to process parameter with index " << paramMD.m_index << ".";
            return false;
        }
    }

    // This code is shared with the CoderBlockParameter
    //
    // Check if the parameters object contains all the information for creating a
    // Configuration object.
    if (Parameters::containConfigurationData(parameters)) {
        if (!ToolboxSingleton::sharedInstance().storeConfiguration(parameters)) {
            wbtError << "Failed to store a Configuration object in the ToolboxSigleton.";
            return false;
        }
        // Save the name of the Configuration block which the processed WBBlock refers to
        if (!parameters.getParameter("ConfBlockName", m_confBlockName)) {
            wbtError << "Failed to read ConfBlockName parameter from the Parameters object "
                     << "that should store Configuration data.";
            return false;
        }
    }

    // Remove the metadata of the parameters already parsed.
    // This is necessary for adding later more metadata and calling again this method
    // (storing again an already stored parameter raises an error).
    m_paramsMetadata.clear();

    return true;
}

std::weak_ptr<wbt::RobotInterface> SimulinkBlockInformation::getRobotInterface() const
{
    // Returns a nullptr if it fails
    return ToolboxSingleton::sharedInstance().getRobotInterface(m_confBlockName);
}

std::weak_ptr<iDynTree::KinDynComputations> SimulinkBlockInformation::getKinDynComputations() const
{
    // Returns a nullptr if it fails
    return ToolboxSingleton::sharedInstance().getKinDynComputations(m_confBlockName);
}
