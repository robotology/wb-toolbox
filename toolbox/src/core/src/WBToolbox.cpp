/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME WBToolbox

#include "Core/Block.h"
#include "Core/Log.h"
#include "Core/Parameter.h"
#include "Core/Parameters.h"
#include "Core/SimulinkBlockInformation.h"
#include "SharedLibrary.h"
#include "SharedLibraryClass.h"

#include <matrix.h>
#include <simstruc.h>
#include <simstruc_types.h>
#include <sl_sample_time_defs.h>
#include <tmwtypes.h>

#include <iostream>
#include <memory>
#include <stdint.h>
#include <stdio.h>
#include <string>
#include <utility>
#include <vector>

std::string platformSpecificLibName(const std::string& library)
{
#if defined(_WIN32)
    return library + "dll";
#elif defined(__linux__)
    return "lib" + library + ".so";
#elif defined(__APPLE__)
    return "lib" + library + ".dylib";
#endif
}

const bool ForwardLogsToStdErr = true;

static void catchLogMessages(bool status, SimStruct* S)
{
    // Initialize static buffers
    const unsigned bufferLen = 1024;

    // Notify warnings
    if (!wbt::Log::getSingleton().getWarnings().empty()) {
        // Get the warnings
        std::string warningMsg = wbt::Log::getSingleton().getWarnings();

        // Trim the message if needed
        if (warningMsg.length() >= bufferLen) {
            warningMsg = warningMsg.substr(0, bufferLen - 1);
        }

        // Forward to Simulink
        char warningBuffer[bufferLen];
        sprintf(warningBuffer, "%s", warningMsg.c_str());
        ssWarning(S, warningBuffer);

        if (ForwardLogsToStdErr) {
            fprintf(stderr, "%s", warningBuffer);
        }

        // Clean the notified warnings
        wbt::Log::getSingleton().clearWarnings();
    }

    // Notify errors
    if (!status) {
        // Get the errors
        std::string errorMsg = wbt::Log::getSingleton().getErrors();

        // Trim the message if needed
        if (errorMsg.length() >= bufferLen) {
            errorMsg = errorMsg.substr(0, bufferLen - 1);
        }

        // Forward to Simulink
        char errorBuffer[bufferLen];
        sprintf(errorBuffer, "%s", errorMsg.c_str());
        ssSetErrorStatus(S, errorBuffer);

        if (ForwardLogsToStdErr) {
            fprintf(stderr, "%s", errorBuffer);
        }

        // Clean the notified errors
        wbt::Log::getSingleton().clearErrors();
        return;
    }
}

// Function: MDL_CHECK_PARAMETERS
#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct* S)
{
    UNUSED_ARG(S);
    // TODO: still to find a way to call Block implementation
}
#endif /*MDL_CHECK_PARAMETERS*/

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct* S, int_T port, const DimsInfo_T* dimsInfo)
{
    // If the port is set as dynamic and the proposed dimension is not dynamic (hence calculated by
    // the signal propagation) accept it
    if (ssGetInputPortWidth(S, port) == DYNAMICALLY_SIZED) {
        if (dimsInfo->width != DYNAMICALLY_SIZED) {
            if (!ssSetInputPortDimensionInfo(S, port, dimsInfo)) {
                wbtError << "Failed to set proposed sizes.";
                catchLogMessages(false, S);
                return;
            }
        }
    }
}

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
static void mdlSetOutputPortDimensionInfo(SimStruct* S, int_T port, const DimsInfo_T* dimsInfo)
{
    // If the port is set as dynamic and the proposed dimension is not dynamic (hence calculated by
    // the signal propagation) accept it
    if (ssGetOutputPortWidth(S, port) == DYNAMICALLY_SIZED) {
        if (dimsInfo->width != DYNAMICALLY_SIZED) {
            if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) {
                wbtError << "Failed to set proposed sizes.";
                catchLogMessages(false, S);
                return;
            }
        }
    }
}

// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, s, states, etc.).
static void mdlInitializeSizes(SimStruct* S)
{
    // Initialize the Log singleton
    wbt::Log::getSingleton().clear();

    if (ssGetSFcnParamsCount(S) < 1) {
        wbtError << "The block type parameter must be specified";
        catchLogMessages(false, S);
        return;
    }

    // Get the class name
    const std::string className(mxArrayToString(ssGetSFcnParam(S, 0)));

    // Get the library name
    const std::string blockLibraryName(mxArrayToString(ssGetSFcnParam(S, 1)));

    // Allocate the block from the Factory.
    // At this stage, the object is just temporary.
    std::unique_ptr<wbt::Block> block;
    {
        // Allocate the factory
        shlibpp::SharedLibraryClassFactory<wbt::Block> factory(
            platformSpecificLibName(blockLibraryName).c_str(), className.c_str());
        if (!factory.isValid()) {
            wbtError << "Factory error: " << shlibpp::Vocab::decode(factory.getStatus())
                     << factory.getLastNativeError();
            catchLogMessages(false, S);
            return;
        }

        // Allocate the block from the factory
        block.reset(factory.create());
    }

    // Notify errors
    if (!block) {
        wbtError << "Could not create an object of type " + className;
        catchLogMessages(false, S);
        return;
    }

    // We cannot save data in PWork during the initializeSizes phase.

    // Two PWorks:
    // 0: pointer to a Block implementation
    // 1: pointer to a BlockInformation implementation
    ssSetNumPWork(S, 2);

    // Setup the block parameters' properties
    ssSetNumSFcnParams(S, block->numberOfParameters());
    ssSetSFcnParamTunable(S, 0, false);
    for (unsigned i = 0; i < ssGetNumSFcnParams(S); ++i) {
        bool tunable = false;
        block->parameterAtIndexIsTunable(i, tunable);
        ssSetSFcnParamTunable(S, i, tunable);
    }

#if defined(MATLAB_MEX_FILE)
    if (ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if (ssGetErrorStatus(S)) {
            return;
        }
    }
    else {
        wbtError << "Number of parameters different from those defined";
        catchLogMessages(false, S);
        return;
    }
#endif

    wbt::SimulinkBlockInformation blockInfo(S);
    bool ok = block->configureSizeAndPorts(&blockInfo);
    catchLogMessages(ok, S);

    if (!ok) {
        return;
    }

    for (auto i = 0; i < ssGetNumInputPorts(S); ++i) {
        // Set explicitly the inputs port to be SS_NOT_REUSABLE_AND_GLOBAL (which actually
        // is already the default value). Since the toolbox supports contiguous input signals,
        // this option should not be changed.
        ssSetInputPortOptimOpts(S, i, SS_NOT_REUSABLE_AND_GLOBAL);
        // Set input signals to be allocated in a contiguous memory storage
        ssSetInputPortRequiredContiguous(S, i, true);
    }

    ssSetNumSampleTimes(S, 1);

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE); //??

    ssSetNumDiscStates(S, block->numberOfDiscreteStates());
    ssSetNumContStates(S, 0); // block->numberOfContinuousStates());

    uint_T options = SS_OPTION_WORKS_WITH_CODE_REUSE | SS_OPTION_EXCEPTION_FREE_CODE
                     | SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION | SS_OPTION_USE_TLC_WITH_ACCELERATOR
                     | SS_OPTION_CALL_TERMINATE_ON_EXIT;
    // also ?
    // SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE

    std::vector<std::string> additionalOptions = block->additionalBlockOptions();

    for (const auto& additionalOption : additionalOptions) {
        double option;
        if (!blockInfo.optionFromKey(additionalOption, option)) {
            wbtError << "Failed to get option from key.";
            catchLogMessages(false, S);
            return;
        }
        // Store the parsed option
        options |= static_cast<uint32_t>(option);
    }

    // Set the options
    ssSetOptions(S, options);
}

// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct* S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

// Function: mdlStart =======================================================
// Abstract:
//   This function is called once at start of model execution. If you
//   have states that should be initialized once, this is the place
//   to do it.
#define MDL_START
static void mdlStart(SimStruct* S)
{
    // Get the class name
    const std::string className(mxArrayToString(ssGetSFcnParam(S, 0)));

    // Get the library name
    const std::string blockLibraryName(mxArrayToString(ssGetSFcnParam(S, 1)));

    // Allocate the block from the Factory and store its pointer in the PWork
    wbt::Block* block;
    {
        // Allocate the factory
        shlibpp::SharedLibraryClassFactory<wbt::Block> factory(
            platformSpecificLibName(blockLibraryName).c_str(), className.c_str());
        if (!factory.isValid()) {
            wbtError << "Factory error: " << shlibpp::Vocab::decode(factory.getStatus())
                     << factory.getLastNativeError();
            catchLogMessages(false, S);
            return;
        }

        // Allocate the block from the factory
        block = factory.create();
    }

    ssSetPWorkValue(S, 0, block);

    // Allocate the BlockInformation object and store its pointer in the PWork
    wbt::BlockInformation* blockInfo = new wbt::SimulinkBlockInformation(S);
    ssSetPWorkValue(S, 1, blockInfo);

    if (!block || !blockInfo) {
        wbtError << "Failed to create objects before storing them in the PWork.";
        catchLogMessages(false, S);
        return;
    }

    // Call the initialize() method
    bool ok = block->initialize(blockInfo);
    catchLogMessages(ok, S);
}

#define MDL_UPDATE
#if defined(MDL_UPDATE) && defined(MATLAB_MEX_FILE)
static void mdlUpdate(SimStruct* S, int_T tid)
{
    UNUSED_ARG(tid);
    if (ssGetNumPWork(S) != 2) {
        wbtError << "PWork should contain two elements.";
        catchLogMessages(false, S);
        return;
    }

    // Get the Block object
    wbt::Block* block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));
    // Get the SimulinkBlockInformation object
    wbt::SimulinkBlockInformation* blockInfo;
    blockInfo = static_cast<wbt::SimulinkBlockInformation*>(ssGetPWorkValue(S, 1));

    if (!block || !blockInfo) {
        wbtError << "Failed to get pointers from the PWork vector.";
        catchLogMessages(false, S);
        return;
    }

    // Call the updateDiscreteState() method
    bool ok = block->updateDiscreteState(blockInfo);
    catchLogMessages(ok, S);
}
#endif

// Initialize the state vectors of this C MEX S-function
#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS) && defined(MATLAB_MEX_FILE)
static void mdlInitializeConditions(SimStruct* S)
{
    if (ssGetNumPWork(S) != 2) {
        wbtError << "PWork should contain two elements.";
        catchLogMessages(false, S);
        return;
    }

    // Get the Block object
    wbt::Block* block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));
    // Get the SimulinkBlockInformation object
    wbt::SimulinkBlockInformation* blockInfo;
    blockInfo = static_cast<wbt::SimulinkBlockInformation*>(ssGetPWorkValue(S, 1));

    if (!block || !blockInfo) {
        wbtError << "Failed to get pointers from the PWork vector.";
        catchLogMessages(false, S);
        return;
    }

    // Call the initializeInitialConditions() method
    bool ok = block->initializeInitialConditions(blockInfo);
    catchLogMessages(ok, S);
}
#endif

#define MDL_DERIVATIVES
#if defined(MDL_DERIVATIVES) && defined(MATLAB_MEX_FILE)
static void mdlDerivatives(SimStruct* /*S*/)
{
    /* Add mdlDerivatives code here */
}
#endif

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct* S, int_T tid)
{
    UNUSED_ARG(tid);
    if (ssGetNumPWork(S) != 2) {
        wbtError << "PWork should contain two elements.";
        catchLogMessages(false, S);
        return;
    }

    // Get the Block object
    wbt::Block* block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));
    // Get the SimulinkBlockInformation object
    wbt::SimulinkBlockInformation* blockInfo;
    blockInfo = static_cast<wbt::SimulinkBlockInformation*>(ssGetPWorkValue(S, 1));

    if (!block || !blockInfo) {
        wbtError << "Failed to get pointers from the PWork vector.";
        catchLogMessages(false, S);
        return;
    }

    // Call the output() method
    bool ok = block->output(blockInfo);
    catchLogMessages(ok, S);
}

static void mdlTerminate(SimStruct* S)
{
    if (!ssGetPWork(S)) {
        return;
    }

    if (ssGetNumPWork(S) != 2) {
        wbtError << "PWork should contain two elements.";
        catchLogMessages(false, S);
        return;
    }

    // Get the Block object
    wbt::Block* block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));
    // Get the SimulinkBlockInformation object
    wbt::SimulinkBlockInformation* blockInfo;
    blockInfo = static_cast<wbt::SimulinkBlockInformation*>(ssGetPWorkValue(S, 1));

    if (block) {
        if (!block->terminate(blockInfo)) {
            wbtError << "Failed to terminate block.";
            catchLogMessages(false, S);
        }
    }
    else {
        wbtWarning << "Failed to get Block pointer from the PWork vector." << std::endl
                   << "Could't terminate block";
    }

    // Delete the resources allocated in the PWork vector;
    delete block;
    delete blockInfo;

    // Clean the PWork vector
    ssSetPWorkValue(S, 0, nullptr);
    ssSetPWorkValue(S, 1, nullptr);
}

#if defined(MATLAB_MEX_FILE)
#define MDL_RTW

template <typename T>
static std::vector<real_T> toRTWNumericVector(const std::vector<T>& vectorInput)
{
    std::vector<real_T> output;
    output.reserve(vectorInput.size());

    output.assign(vectorInput.begin(), vectorInput.end());
    return output;
}

static std::string toRTWStringVector(const std::vector<std::string>& stringInput)
{
    std::string output;

    for (unsigned i = 0; i < stringInput.size(); ++i) {
        if (i == 0) {
            output += "[\"" + stringInput[i] + "\"";
        }
        else {
            output += ", \"" + stringInput[i] + "\"";
        }
    }
    output += "]";
    return output;
}

const std::pair<std::string, std::string> parameterTypeToString(const wbt::ParameterType& type)
{
    switch (type) {
        case wbt::ParameterType::INT:
            return {"ParameterType::INT", "int"};
        case wbt::ParameterType::BOOL:
            return {"ParameterType::BOOL", "bool"};
        case wbt::ParameterType::DOUBLE:
            return {"ParameterType::DOUBLE", "double"};
        case wbt::ParameterType::STRING:
            return {"ParameterType::STRING", "std::string"};
        case wbt::ParameterType::CELL_INT:
            return {"ParameterType::CELL_INT", "int"};
        case wbt::ParameterType::CELL_BOOL:
            return {"ParameterType::CELL_BOOL", "bool"};
        case wbt::ParameterType::CELL_DOUBLE:
            return {"ParameterType::CELL_DOUBLE", "double"};
        case wbt::ParameterType::CELL_STRING:
            return {"ParameterType::CELL_STRING", "std::string"};
        case wbt::ParameterType::STRUCT_INT:
            return {"ParameterType::STRUCT_INT", "int"};
        case wbt::ParameterType::STRUCT_BOOL:
            return {"ParameterType::STRUCT_BOOL", "bool"};
        case wbt::ParameterType::STRUCT_DOUBLE:
            return {"ParameterType::STRUCT_DOUBLE", "double"};
        case wbt::ParameterType::STRUCT_STRING:
            return {"ParameterType::STRUCT_STRING", "std::string"};
        case wbt::ParameterType::STRUCT_CELL_INT:
            return {"ParameterType::STRUCT_CELL_INT", "int"};
        case wbt::ParameterType::STRUCT_CELL_BOOL:
            return {"ParameterType::STRUCT_CELL_BOOL", "bool"};
        case wbt::ParameterType::STRUCT_CELL_DOUBLE:
            return {"ParameterType::STRUCT_CELL_DOUBLE", "double"};
        case wbt::ParameterType::STRUCT_CELL_STRING:
            return {"ParameterType::STRUCT_CELL_STRING", "std::string"};
    }
}

template <typename T>
static bool writeParameterToRTW(const wbt::Parameter<T> param, SimStruct* S)
{
    if (param.getMetadata().cols == wbt::ParameterMetadata::DynamicSize
        || param.getMetadata().rows == wbt::ParameterMetadata::DynamicSize) {
        wbtError << "Storing in the rtw file dynamically-sized parameters is not supported.";
        return false;
    }

    if (param.isScalar()) {
        return ssWriteRTWParamSettings(
            S,
            8,
            SSWRITE_VALUE_NUM,
            "index",
            static_cast<real_T>(param.getMetadata().index),
            SSWRITE_VALUE_QSTR,
            "name",
            param.getMetadata().name.c_str(),
            SSWRITE_VALUE_NUM,
            "isScalar",
            static_cast<real_T>(param.isScalar()),
            SSWRITE_VALUE_NUM,
            "rows",
            static_cast<real_T>(param.getMetadata().rows),
            SSWRITE_VALUE_NUM,
            "cols",
            static_cast<real_T>(param.getMetadata().cols),
            SSWRITE_VALUE_QSTR,
            "type",
            parameterTypeToString(param.getMetadata().type).first.c_str(),
            SSWRITE_VALUE_QSTR,
            "storage",
            parameterTypeToString(param.getMetadata().type).second.c_str(),
            SSWRITE_VALUE_NUM,
            "valueScalar",
            static_cast<real_T>(param.getScalarParameter()));
    }
    else {
        const std::vector<real_T> vectorRealT = toRTWNumericVector(param.getVectorParameter());
        return ssWriteRTWParamSettings(
            S,
            8,
            SSWRITE_VALUE_NUM,
            "index",
            static_cast<real_T>(param.getMetadata().index),
            SSWRITE_VALUE_QSTR,
            "name",
            param.getMetadata().name.c_str(),
            SSWRITE_VALUE_NUM,
            "isScalar",
            static_cast<real_T>(param.isScalar()),
            SSWRITE_VALUE_NUM,
            "rows",
            static_cast<real_T>(param.getMetadata().rows),
            SSWRITE_VALUE_NUM,
            "cols",
            static_cast<real_T>(param.getMetadata().cols),
            SSWRITE_VALUE_QSTR,
            "type",
            parameterTypeToString(param.getMetadata().type).first.c_str(),
            SSWRITE_VALUE_QSTR,
            "storage",
            parameterTypeToString(param.getMetadata().type).second.c_str(),
            SSWRITE_VALUE_VECT,
            "valueVector",
            vectorRealT.data(),
            param.getVectorParameter().size());
    }
}

// Specialize the template for std::string
template <>
bool writeParameterToRTW(const wbt::Parameter<std::string> param, SimStruct* S)
{
    if (param.getMetadata().cols == wbt::ParameterMetadata::DynamicSize
        || param.getMetadata().rows == wbt::ParameterMetadata::DynamicSize) {
        wbtError << "Storing in the rtw file dynamically-sized parameters is not supported.";
        return false;
    }

    if (param.isScalar()) {
        return ssWriteRTWParamSettings(
            S,
            8,
            SSWRITE_VALUE_NUM,
            "index",
            static_cast<real_T>(param.getMetadata().index),
            SSWRITE_VALUE_QSTR,
            "name",
            param.getMetadata().name.c_str(),
            SSWRITE_VALUE_NUM,
            "isScalar",
            static_cast<real_T>(param.isScalar()),
            SSWRITE_VALUE_NUM,
            "rows",
            static_cast<real_T>(param.getMetadata().rows),
            SSWRITE_VALUE_NUM,
            "cols",
            static_cast<real_T>(param.getMetadata().cols),
            SSWRITE_VALUE_QSTR,
            "type",
            parameterTypeToString(param.getMetadata().type).first.c_str(),
            SSWRITE_VALUE_QSTR,
            "storage",
            parameterTypeToString(param.getMetadata().type).second.c_str(),
            SSWRITE_VALUE_QSTR,
            "valueScalar",
            param.getScalarParameter().c_str());
    }
    else {
        const std::string serializedVectorOfStrings = toRTWStringVector(param.getVectorParameter());
        return ssWriteRTWParamSettings(
            S,
            8,
            SSWRITE_VALUE_NUM,
            "index",
            static_cast<real_T>(param.getMetadata().index),
            SSWRITE_VALUE_QSTR,
            "name",
            param.getMetadata().name.c_str(),
            SSWRITE_VALUE_NUM,
            "isScalar",
            static_cast<real_T>(param.isScalar()),
            SSWRITE_VALUE_NUM,
            "rows",
            static_cast<real_T>(param.getMetadata().rows),
            SSWRITE_VALUE_NUM,
            "cols",
            static_cast<real_T>(param.getMetadata().cols),
            SSWRITE_VALUE_QSTR,
            "type",
            parameterTypeToString(param.getMetadata().type).first.c_str(),
            SSWRITE_VALUE_QSTR,
            "storage",
            parameterTypeToString(param.getMetadata().type).second.c_str(),
            SSWRITE_VALUE_VECT_STR,
            "valueVector",
            serializedVectorOfStrings.c_str(),
            param.getMetadata().cols);
    }
}

static bool writeRTW(SimStruct* S, const wbt::Parameters& params)
{
    // RTW Parameters record metadata
    // ==============================

    // The first entry in the parameter record (SFcnParamSettings[0]) contains
    // information useful to parse the others

    // Get the number or parameters
    const unsigned numberOfParameters = params.getNumberOfParameters();

    // Get the class name
    std::string className;
    params.getParameter("className", className);

    // Create the record
    ssWriteRTWParamSettings(S,
                            2,
                            SSWRITE_VALUE_NUM,
                            "numberOfParameters",
                            static_cast<real_T>(numberOfParameters),
                            SSWRITE_VALUE_QSTR,
                            "className",
                            className.c_str());

    // RTW Parameters
    // ==============

    bool ok = true;

    for (const auto& param : params.getIntParameters()) {
        ok = ok && writeParameterToRTW(param, S);
    }

    for (const auto& param : params.getBoolParameters()) {
        ok = ok && writeParameterToRTW(param, S);
    }

    for (const auto& param : params.getDoubleParameters()) {
        ok = ok && writeParameterToRTW(param, S);
    }

    for (const auto& param : params.getStringParameters()) {
        ok = ok && writeParameterToRTW(param, S);
    }

    if (!ok) {
        wbtError << "Failed to write parameters to RTW file.";
        catchLogMessages(false, S);
        return false;
    }

    return true;
}

static void mdlRTW(SimStruct* S)
{
    if (ssGetNumPWork(S) > 0 && ssGetPWork(S)) {

        // Get the block object from the PWork
        wbt::Block* block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));

        bool ok;
        wbt::Parameters params;

        if (!block) {
            wbtError << "Unable to get the class from the PWork vector.";
            catchLogMessages(false, S);
            return;
        }

        // Get the parameters from the block
        ok = block->getParameters(params);
        catchLogMessages(ok, S);
        if (!ok)
            return;

        // Use parameters metadata to populate the rtw file used by the coder
        ok = writeRTW(S, params);
        catchLogMessages(ok, S);
        if (!ok)
            return;

        // Store the PWork vector in the rtw file.
        ok = ssWriteRTWWorkVect(S, "PWork", 1, "blockPWork", ssGetNumPWork(S));
        catchLogMessages(ok, S);
        if (!ok)
            return;
    }
}
#endif

// Required S-function trailer
#ifdef MATLAB_MEX_FILE /* Is this file being compiled as a MEX-file? */
#include <simulink.c> /* MEX-file interface mechanism */
#else
#include "cg_sfun.h" /* Code generation registration function */
#endif
