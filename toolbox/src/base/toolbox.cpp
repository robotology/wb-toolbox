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

#include <string>
#include <yarp/os/LogStream.h>

// Function: MDL_CHECK_PARAMETERS
#define MDL_CHECK_PARAMETERS
#if defined(MDL_CHECK_PARAMETERS) && defined(MATLAB_MEX_FILE)
static void mdlCheckParameters(SimStruct *S)
{
    //TODO: still to find a way to call Block implementation
}
#endif  /*MDL_CHECK_PARAMETERS*/


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
    wbit::Block *block = wbit::Block::instantiateBlockWithClassName(className);

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

    wbit::Error error;
    if (!block->configureSizeAndPorts(S, &error)) {
        sprintf(errorBuffer, "%s", error.message.substr(0, 511).c_str());
        ssSetErrorStatus(S, errorBuffer);
        return;
    }

    delete block;

    ssSetNumSampleTimes(S, 1);

    ssSetSimStateCompliance(S, USE_CUSTOM_SIM_STATE); //??

    ssSetOptions(S,
                 SS_OPTION_WORKS_WITH_CODE_REUSE |
                 SS_OPTION_EXCEPTION_FREE_CODE |
                 SS_OPTION_ALLOW_INPUT_SCALAR_EXPANSION |
                 SS_OPTION_USE_TLC_WITH_ACCELERATOR |
                 SS_OPTION_CALL_TERMINATE_ON_EXIT);

}

// Function: mdlInitializeSampleTimes =========================================
// Abstract:
//   This function is used to specify the sample time(s) for your
//   S-function. You must register the same number of sample times as
//   specified in ssSetNumSampleTimes.
static void mdlInitializeSampleTimes(SimStruct *S)
{
    // The sampling time of this SFunction must be inherited so that the Soft Real Time sblock can be used.
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    // ssSetSampleTime(S, 0, 10.0);
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
    wbit::Block *block = wbit::Block::instantiateBlockWithClassName(className);
    ssSetPWorkValue(S, 0, block);

    if (ssGetNumPWork(S) > 0) {
        wbit::Block *block = static_cast<wbit::Block*>(ssGetPWorkValue(S, 0));
        wbit::Error error;
        if (!block || !block->initialize(S, &error)) {
            yError() << "[mdlStart]" << error.message;
            static char errorBuffer[1024];
            sprintf(errorBuffer, "[mdlStart]%s", error.message.substr(0, 1023 - strlen("[mdlStart]")).c_str());
            ssSetErrorStatus(S, errorBuffer);
        }
    }
}

// Function: mdlOutputs =======================================================
// Abstract:
//   In this function, you compute the outputs of your S-function
//   block.
static void mdlOutputs(SimStruct *S, int_T tid)
{
    UNUSED_ARG(tid);
    if (ssGetNumPWork(S) > 0) {
        wbit::Block *block = static_cast<wbit::Block*>(ssGetPWorkValue(S, 0));
        wbit::Error error;
        if (!block || !block->output(S, &error)) {
            static char errorBuffer[1024];
            sprintf(errorBuffer, "[mdlOutputs]%s", error.message.substr(0, 1023 - strlen("[mdlOutputs]")).c_str());
            ssSetErrorStatus(S, errorBuffer);
        }
    }

}

static void mdlTerminate(SimStruct *S)
{
    if (ssGetNumPWork(S) > 0 && ssGetPWork(S)) {
        wbit::Block *block = static_cast<wbit::Block*>(ssGetPWorkValue(S, 0));
        wbit::Error error;
        if (block) {
            if (block->terminate(S, &error)) {
                delete block;
                ssSetPWorkValue(S, 0, NULL);
            } else {
                static char errorBuffer[1024];
                sprintf(errorBuffer, "[mdlTerminate]%s", error.message.substr(0, 1023 - strlen("[mdlTerminate]")).c_str());
                ssSetErrorStatus(S, errorBuffer);

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
