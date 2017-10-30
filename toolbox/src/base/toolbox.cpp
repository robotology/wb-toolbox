/*
 * Copyright (C) 2013-2015 Robotics, Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
 * Author: Jorhabib Eljaik Gomez, Francesco Romano
 * email: jorhabib.eljaik@iit.it, francesco.romano@iit.it
 *
 * The development of this software was supported by the FP7 EU project
 * CoDyCo (No. 600716 ICT 2011.2.1 Cognitive Systems and Robotics (b))
 * http://www.codyco.eu
 *
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#define S_FUNCTION_LEVEL 2
#define S_FUNCTION_NAME WBToolbox

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"

#include "toolbox.h"
#include "Block.h"
#include "Log.h"
#include "SimulinkBlockInformation.h"

#include <string>
#include <yarp/os/LogStream.h>

static void catchLogMessages(bool status, SimStruct *S, std::string prefix)
{
    // Initialize static buffers
    const unsigned bufferLen = 1024;
    static char errorBuffer[bufferLen];
    static char warningBuffer[bufferLen];

    // Notify warnings
    if (!wbt::Log::getSingleton().getWarnings().empty()) {
        // Get the singleton
        wbt::Log& log = wbt::Log::getSingleton();

        // Handle the prefix
        std::string warningMsg;
        if (!prefix.empty()) {
            log.setPrefix(prefix);
            warningMsg = log.getWarnings();
            log.resetPrefix();
        }
        else {
            warningMsg = log.getWarnings();
        }

        // Trim the message if needed
        if (warningMsg.length() >= bufferLen) {
            warningMsg = warningMsg.substr(0, bufferLen-1);
        }

        // Forward to Simulink
        sprintf(warningBuffer, "%s", warningMsg.c_str());
        ssWarning(S, warningBuffer);
        log.clearWarnings();
    }

    // Notify errors
    if (!status) {
        // Get the singleton
        wbt::Log& log = wbt::Log::getSingleton();

        // Handle the prefix
        std::string errorMsg;
        if (!prefix.empty()) {
            log.setPrefix(prefix);
            errorMsg = log.getErrors();
            log.resetPrefix();
        }
        else {
            errorMsg = log.getErrors();
        }

        // Trim the message if needed
        if (errorMsg.length() >= bufferLen) {
            errorMsg = errorMsg.substr(0, bufferLen-1);
        }

        // Forward to Simulink
        sprintf(errorBuffer, "%s", errorMsg.c_str());
        ssSetErrorStatus(S, errorBuffer);
        return;
    }
}

// Function: MDL_CHECK_PARAMETERS
#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    UNUSED_ARG(S);
    //TODO: still to find a way to call Block implementation
}
#endif  /*MDL_CHECK_PARAMETERS*/

#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct *S, int_T port,
                                         const DimsInfo_T *dimsInfo)
{
    //TODO: for now accept the proposed size.
    //If we want to change the behaviour we have to implement some callbacks
    ssSetInputPortDimensionInfo(S, port, dimsInfo);
}

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
static void mdlSetOutputPortDimensionInfo(SimStruct *S, int_T port,
                                         const DimsInfo_T *dimsInfo)
{
    //TODO: for now accept the proposed size.
    //If we want to change the behaviour we have to implement some callbacks
    ssSetOutputPortDimensionInfo(S, port, dimsInfo);
}

// Function: mdlInitializeSizes ===============================================
// Abstract:
//    The sizes information is used by Simulink to determine the S-function
//    block's characteristics (number of inputs, s, states, etc.).
static void mdlInitializeSizes(SimStruct *S)
{
    // Initialize the Log singleton
    wbt::Log::getSingleton().clear();

    if (ssGetSFcnParamsCount(S) < 1) {
        wbt::Log::getSingleton().error("The block type parameter must be specified");
        catchLogMessages(false, S, "\n[" + std::string(__func__) + "]");
        return;
    }
    char *classNameStr = mxArrayToString(ssGetSFcnParam(S, 0));
    std::string className(classNameStr);
    mxFree(classNameStr);
    wbt::Block *block = wbt::Block::instantiateBlockWithClassName(className);

    //We cannot save data in PWork during the initializeSizes phase
    ssSetNumPWork(S, 1);

    // Notify errors
    if (!block) {
        wbt::Log::getSingleton().error("Could not create an object of type " + className);
        catchLogMessages(false, S, "\n[" + std::string(__func__) + "]");
        return;
    }

    ssSetNumSFcnParams(S, 1 + block->numberOfParameters());
    ssSetSFcnParamTunable(S, 0, false);
    for (unsigned i = 1; i < ssGetNumSFcnParams(S); ++i) {
        bool tunable = false;
        block->parameterAtIndexIsTunable(i - 1, tunable);
        ssSetSFcnParamTunable(S, i, tunable);
    }


#if defined(MATLAB_MEX_FILE)
    if(ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)) {
        mdlCheckParameters(S);
        if(ssGetErrorStatus(S)) {
            return;
        }
    } else {
        wbt::Log::getSingleton().error("Number of parameters different from those defined");
        catchLogMessages(false, S, "\n[" + std::string(__func__) + "]");
        return;
    }
#endif

    wbt::SimulinkBlockInformation blockInfo(S);
    bool ok = block->configureSizeAndPorts(&blockInfo);
    catchLogMessages(ok, S, "\n[" + std::string(__func__) + "]");

    ssSetNumSampleTimes(S, 1);

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE); //??

    ssSetNumDiscStates(S, block->numberOfDiscreteStates());
    ssSetNumContStates(S, 0);//block->numberOfContinuousStates());

    uint_T options =
    SS_OPTION_WORKS_WITH_CODE_REUSE |
    SS_OPTION_EXCEPTION_FREE_CODE |
    SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION |
    SS_OPTION_USE_TLC_WITH_ACCELERATOR |
    SS_OPTION_CALL_TERMINATE_ON_EXIT;
    //also ?
    //SS_OPTION_RUNTIME_EXCEPTION_FREE_CODE

    std::vector<std::string> additionalOptions = block->additionalBlockOptions();

    for (auto additionalOption: additionalOptions) {
        double option;
        if (blockInfo.optionFromKey(additionalOption, option)) {
            options |= static_cast<uint32_t>(option);
        }
    }
    ssSetOptions(S, options);

    delete block;
}

// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
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
static void mdlStart(SimStruct *S)
{
    char *classNameStr = mxArrayToString(ssGetSFcnParam(S, 0));
    std::string className(classNameStr);
    mxFree(classNameStr);
    wbt::Block *block = wbt::Block::instantiateBlockWithClassName(className);
    ssSetPWorkValue(S, 0, block);

    wbt::SimulinkBlockInformation blockInfo(S);
    bool ok = false;
    if (block) {
        ok = block->initialize(&blockInfo);
    }
    catchLogMessages(ok, S, "\n[" + std::string(__func__) + "]");
}


#define MDL_UPDATE
#if defined(MDL_UPDATE) && defined(MATLAB_MEX_FILE)
static void mdlUpdate(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    if (ssGetNumPWork(S) > 0) {
        wbt::Block *block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));

        wbt::SimulinkBlockInformation blockInfo(S);
        bool ok = false;
        if (block) {
            ok = block->updateDiscreteState(&blockInfo);
        }
        catchLogMessages(ok, S, "\n[" + std::string(__func__) + "]");
    }
}
#endif

//Initialize the state vectors of this C MEX S-function
#define MDL_INITIALIZE_CONDITIONS
#if defined(MDL_INITIALIZE_CONDITIONS) && defined(MATLAB_MEX_FILE)
static void mdlInitializeConditions(SimStruct *S)
{
    if (ssGetNumPWork(S) > 0) {
        wbt::Block *block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));

        wbt::SimulinkBlockInformation blockInfo(S);
        bool ok = false;
        if (block) {
            ok = block->initializeInitialConditions(&blockInfo);
        }
        catchLogMessages(ok, S, "\n[" + std::string(__func__) + "]");
    }
}
#endif


#define MDL_DERIVATIVES
#if defined(MDL_DERIVATIVES) && defined(MATLAB_MEX_FILE)
static void mdlDerivatives(SimStruct *S)
{
    /* Add mdlDerivatives code here */
}
#endif


// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    if (ssGetNumPWork(S) > 0) {
        wbt::Block *block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));

        wbt::SimulinkBlockInformation blockInfo(S);
        bool ok = false;
        if (block) {
            ok = block->output(&blockInfo);
        }
        catchLogMessages(ok, S, "\n[" + std::string(__func__) + "]");
    }
}

static void mdlTerminate(SimStruct *S)
{
    if (ssGetNumPWork(S) > 0 && ssGetPWork(S)) {
        wbt::Block *block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));

        wbt::SimulinkBlockInformation blockInfo(S);
        bool ok = false;
        if (block) {
            ok = block->terminate(&blockInfo);
        }
        catchLogMessages(ok, S, "\n[" + std::string(__func__) + "]");

        if (ok) {
            delete block;
            ssSetPWorkValue(S, 0, NULL);
        }
    }
}

// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
