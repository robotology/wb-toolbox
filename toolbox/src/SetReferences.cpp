#include "SetReferences.h"

#include "Error.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>

namespace wbt {

    std::string SetReferences::ClassName = "SetReferences";

    SetReferences::SetReferences()
    : m_references(0)
    , m_controlMode(wbi::CTRL_MODE_UNKNOWN)
    , m_fullControl(true)
    , m_resetControlMode(true) {}

    unsigned SetReferences::numberOfParameters()
    {
        // 1 - Control Type
        // 2 - Full/Sublist type
        // 3 - (only if sublist) controlled joints
        return WBIBlock::numberOfParameters() + 3;
    }

    bool SetReferences::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(S, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        m_fullControl = static_cast<int>(mxGetScalar(ssGetSFcnParam(S,WBIBlock::numberOfParameters() + 2))) == 1;

        if (!m_fullControl) {
            //sublist
            std::string controlledList;
            if (!Block::readStringParameterAtIndex(S, WBIBlock::numberOfParameters() + 3, controlledList)) {
                if (error) error->message = "Could not read control type parameter";
                return false;
            }
            const yarp::os::Property * controlledListProperty = WBInterface::sharedInstance().currentConfiguration();

            wbi::IDList idList;
            WBInterface::wbdIDListFromConfigPropAndList(*controlledListProperty,
                                                        controlledList, idList);
            dofs = idList.size();
        }

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

        // Reading the control type
        std::string controlType;
        if (!Block::readStringParameterAtIndex(S, WBIBlock::numberOfParameters() + 1, controlType)) {
            if (error) error->message = "Could not read control type parameter";
            return false;
        }

        m_controlMode = wbi::CTRL_MODE_UNKNOWN;
        if (controlType == "Position") {
            m_controlMode = wbi::CTRL_MODE_POS;
        } else if (controlType == "Position Direct") {
            m_controlMode = wbi::CTRL_MODE_DIRECT_POSITION;
        } else if (controlType == "Velocity") {
            m_controlMode = wbi::CTRL_MODE_VEL;
        } else if (controlType == "Torque") {
            m_controlMode = wbi::CTRL_MODE_TORQUE;
        } else if (controlType == "Open Loop") {
            m_controlMode = wbi::CTRL_MODE_MOTOR_PWM;
        } else {
            if (error) error->message = "Control Mode not supported";
            return false;
        }

        //Read if full or sublist control
        m_fullControl = static_cast<int>(mxGetScalar(ssGetSFcnParam(S,WBIBlock::numberOfParameters() + 2))) == 1;

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        if (!m_fullControl) {
            //sublist
            std::string controlledList;
            if (!Block::readStringParameterAtIndex(S, WBIBlock::numberOfParameters() + 3, controlledList)) {
                if (error) error->message = "Could not read control type parameter";
                return false;
            }
            const yarp::os::Property * controlledListProperty = WBInterface::sharedInstance().currentConfiguration();

            wbi::IDList idList;
            bool result = WBInterface::wbdIDListFromConfigPropAndList(*controlledListProperty,
                                                                      controlledList, idList);
            wbi::IDList fullList = WBInterface::sharedInstance().interface()->getJointList();
            m_controlledJoints.clear();
            m_controlledJoints.reserve(idList.size());
            if (result) {
                for (int i = 0; i < idList.size(); i++) {
                    int index;
                    if (fullList.idToIndex(idList.at(i), index))
                        m_controlledJoints.push_back(index);
                    else
                        std::cerr << "Joint " << idList.at(i).toString() << " not found\n";
                }
            }
            dofs = idList.size();
        }
        m_references = new double[dofs];

        m_resetControlMode = true;
        return m_references;
    }

    bool SetReferences::terminate(SimStruct *S, wbt::Error *error)
    {
        wbi::wholeBodyInterface * const interface = WBInterface::sharedInstance().interface();
        if (interface) {
            if (m_fullControl) {
                interface->setControlMode(wbi::CTRL_MODE_POS);
            } else {
                for (int i = 0; i < m_controlledJoints.size(); i++) {
                    interface->setControlMode(wbi::CTRL_MODE_POS, 0, m_controlledJoints[i]);
                }
            }
        }
        if (m_references) {
            delete [] m_references;
            m_references = 0;
        }
        return WBIBlock::terminate(S, error);
    }

    bool SetReferences::initializeInitialConditions(SimStruct */*S*/, wbt::Error */*error*/)
    {
        //Simply reset the variable m_resetControlMode
        //It will be read at the first cycle of output
        m_resetControlMode = true;
        return true;
    }

    bool SetReferences::output(SimStruct *S, wbt::Error */*error*/)
    {
        //get input
        wbi::wholeBodyInterface * const interface = WBInterface::sharedInstance().interface();
        if (interface) {
            if (m_resetControlMode) {
                m_resetControlMode = false;

                //now switch control mode
                if (m_fullControl) {
                    interface->setControlMode(m_controlMode);
                } else {
                    for (int i = 0; i < m_controlledJoints.size(); i++) {
                        interface->setControlMode(m_controlMode, 0, m_controlledJoints[i]);
                    }
                }
            }

            InputRealPtrsType references = ssGetInputPortRealSignalPtrs(S, 0);
            for (unsigned i = 0; i < ssGetInputPortWidth(S, 0); ++i) {
                m_references[i] = *references[i];
            }
            if (m_fullControl) {
                interface->setControlReference(m_references);
            } else {
                for (int i = 0; i < m_controlledJoints.size(); i++) {
                    interface->setControlReference(&m_references[i], m_controlledJoints[i]);
                }
            }
            return true;
        }
        return false;
    }
}
