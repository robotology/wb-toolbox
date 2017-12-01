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
#define S_FUNCTION_NAME WBToolbox2

// Need to include simstruc.h for the definition of the SimStruct and
// its associated macro definitions.
#include "simstruc.h"

#include "toolbox.h"
#include "Block.h"
#include "SimulinkBlockInformation.h"

#include <string>
#include <yarp/os/LogStream.h>

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
    static char errorBuffer[512];
    if (ssGetSFcnParamsCount(S) < 1) {
        sprintf(errorBuffer, "%s", "The block type parameter must be specified");
        ssSetErrorStatus(S, errorBuffer);
        return;
    }
    char *classNameStr = mxArrayToString(ssGetSFcnParam(S, 0));
    std::string className(classNameStr);
    mxFree(classNameStr);
    wbt::Block *block = wbt::Block::instantiateBlockWithClassName(className);

    //We cannot save data in PWork during the initializeSizes phase
    ssSetNumPWork(S, 1);
    
    if (!block) {
        sprintf(errorBuffer, "Could not create an object of type %s", className.c_str());
        ssSetErrorStatus(S, errorBuffer);
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
    if(ssGetNumSFcnParams(S) == ssGetSFcnParamsCount(S)){
        mdlCheckParameters(S);
        if(ssGetErrorStatus(S)){
            return;
        }
    } else{
        sprintf(errorBuffer, "%s", "Number of parameters different from those defined");
        ssSetErrorStatus(S, errorBuffer);
        return;
    }
#endif

    wbt::Error error;
    wbt::SimulinkBlockInformation blockInfo(S);
    if (!block->configureSizeAndPorts(&blockInfo, &error)) {
        sprintf(errorBuffer, "%s", error.message.substr(0, 511).c_str());
        ssSetErrorStatus(S, errorBuffer);
        return;
    }

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

    for (std::vector<std::string>::const_iterator it = additionalOptions.begin();
         it < additionalOptions.end(); ++it) {
        wbt::Data option;
        if (blockInfo.optionFromKey(*it, option)) {
            options |= option.uint32Data();
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

    wbt::Error error;
    wbt::SimulinkBlockInformation blockInfo(S);
    if (!block || !block->initialize(&blockInfo, &error)) {
        yError() << "[mdlStart]" << error.message;
        static char errorBuffer[1024];
        sprintf(errorBuffer, "[mdlStart]%s", error.message.substr(0, 1023 - strlen("[mdlStart]")).c_str());
        ssSetErrorStatus(S, errorBuffer);
    }

}


#define MDL_UPDATE
#if defined(MDL_UPDATE) && defined(MATLAB_MEX_FILE)
static void mdlUpdate(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    if (ssGetNumPWork(S) > 0) {
        wbt::Block *block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));
        wbt::Error error;
        wbt::SimulinkBlockInformation blockInfo(S);
        if (!block || !block->updateDiscreteState(&blockInfo, &error)) {
            static char errorBuffer[1024];
            sprintf(errorBuffer, "[mdlOutputs]%s", error.message.substr(0, 1023 - strlen("[mdlOutputs]")).c_str());
            ssSetErrorStatus(S, errorBuffer);
        }
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
        wbt::Error error;
        wbt::SimulinkBlockInformation blockInfo(S);
        if (!block || !block->initializeInitialConditions(&blockInfo, &error)) {
            static char errorBuffer[1024];
            sprintf(errorBuffer, "[mdlInitializeConditions]%s", error.message.substr(0, 1023 - strlen("[mdlInitializeConditions]")).c_str());
            ssSetErrorStatus(S, errorBuffer);
        }
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
        wbt::Error error;
        wbt::SimulinkBlockInformation blockInfo(S);
        if (!block || !block->output(&blockInfo, &error)) {
            static char errorBuffer[1024];
            sprintf(errorBuffer, "[mdlOutputs]%s", error.message.substr(0, 1023 - strlen("[mdlOutputs]")).c_str());
            ssSetErrorStatus(S, errorBuffer);
        }
    }

}

static void mdlTerminate(SimStruct *S)
{
    if (ssGetNumPWork(S) > 0 && ssGetPWork(S)) {
        wbt::Block *block = static_cast<wbt::Block*>(ssGetPWorkValue(S, 0));
        wbt::Error error;
        wbt::SimulinkBlockInformation blockInfo(S);
        if (block) {
            if (block->terminate(&blockInfo, &error)) {
                delete block;
                ssSetPWorkValue(S, 0, NULL);
            } else {
                static char errorBuffer[1024];
                sprintf(errorBuffer, "[mdlTerminate]%s", error.message.substr(0, 1023 - strlen("[mdlTerminate]")).c_str());
                ssSetErrorStatus(S, errorBuffer);
            }
        }
    }

}

// Required S-function trailer
#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
