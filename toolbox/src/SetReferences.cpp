#include "SetReferences.h"

#include "Error.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>

namespace wbt {

    std::string SetReferences::ClassName = "SetReferences";

    SetReferences::SetReferences()
    : m_references(0)
    , m_firstRun(true)
    , m_controlMode(wbi::CTRL_MODE_UNKNOWN) {}

    unsigned SetReferences::numberOfParameters()
    {
        return WBIBlock::numberOfParameters() + 1;
    }

    bool SetReferences::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(S, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        // Specify I/O
        if (!ssSetNumInputPorts (S, 1)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }

        bool success = ssSetInputPortVectorDimension(S, 0, dofs);
        ssSetInputPortDataType (S, 0, SS_DOUBLE);
        ssSetInputPortDirectFeedThrough (S, 0, 1);

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        // Output port:
        if (!ssSetNumOutputPorts (S, 0)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        return true;
    }

    bool SetReferences::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIBlock::initialize(S, error)) return false;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        m_references = new double[dofs];

        // Reading the control type
        std::string controlType;
        if (!Block::readStringParameterAtIndex(S, WBIBlock::numberOfParameters() + 1, controlType)) {
            if (error) error->message = "Could not read control type parameter";
            return false;
        }
        
        wbi::ControlMode m_controlMode = wbi::CTRL_MODE_UNKNOWN;
        if (controlType == "Position") {
            m_controlMode = wbi::CTRL_MODE_POS;
        } else if (controlType == "Position Direct") {
            m_controlMode = wbi::CTRL_MODE_DIRECT_POSITION;
        } else if (controlType == "Velocity") {
            m_controlMode = wbi::CTRL_MODE_VEL;
        } else if (controlType == "Torque") {
            m_controlMode = wbi::CTRL_MODE_TORQUE;
        } else {
            if (error) error->message = "Control Mode not supported";
            return false;
        }

        m_firstRun = true;
        return m_references;
    }

    bool SetReferences::terminate(SimStruct *S, wbt::Error *error)
    {
        wbi::wholeBodyInterface * const interface = WBInterface::sharedInstance().interface();
        if (interface) {
            interface->setControlMode(wbi::CTRL_MODE_POS);
        }
        if (m_references) {
            delete [] m_references;
            m_references = 0;
        }
        return WBIBlock::terminate(S, error);
    }

    bool SetReferences::output(SimStruct *S, wbt::Error */*error*/)
    {
        //get input
        wbi::wholeBodyInterface * const interface = WBInterface::sharedInstance().interface();
        if (interface) {
            if (m_firstRun) {
                m_firstRun = false;
                interface->setControlMode(m_controlMode); //should I return an error here?
            }
            InputRealPtrsType references = ssGetInputPortRealSignalPtrs(S, 0);
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 0); ++i) {
                m_references[i] = *references[i];
            }
            interface->setControlReference(m_references);
            return true;
        }
        return false;
    }
}
