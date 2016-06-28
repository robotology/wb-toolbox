#include "SetReferences.h"

#include "Error.h"
#include "WBInterface.h"
#include "BlockInformation.h"
#include "Signal.h"
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

    bool SetReferences::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(blockInfo, error)) {
            return false;
        }

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();

        m_fullControl = blockInfo->getScalarParameterAtIndex(WBIBlock::numberOfParameters() + 2).booleanData();

        if (!m_fullControl) {
            //sublist
            std::string controlledList;
            if (!blockInfo->getStringParameterAtIndex(WBIBlock::numberOfParameters() + 3, controlledList)) {
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
        if (!blockInfo->setNumberOfInputPorts(1)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }

        bool success = blockInfo->setInputPortVectorSize(0, dofs);
        blockInfo->setInputPortType(0, PortDataTypeDouble);

        if (!success) {
            if (error) error->message = "Failed to configure input ports";
            return false;
        }

        // Output port:
        if (!blockInfo->setNumberOfOuputPorts(0)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        return true;
    }

    bool SetReferences::initialize(BlockInformation *blockInfo, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIBlock::initialize(blockInfo, error)) return false;

        // Reading the control type
        std::string controlType;
        if (!blockInfo->getStringParameterAtIndex(WBIBlock::numberOfParameters() + 1, controlType)) {
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
        m_fullControl = blockInfo->getScalarParameterAtIndex(WBIBlock::numberOfParameters() + 2).booleanData();

        unsigned dofs = WBInterface::sharedInstance().numberOfDoFs();
        if (!m_fullControl) {
            //sublist
            std::string controlledList;
            if (!blockInfo->getStringParameterAtIndex(WBIBlock::numberOfParameters() + 3, controlledList)) {
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

    bool SetReferences::terminate(BlockInformation *blockInfo, wbt::Error *error)
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
        return WBIBlock::terminate(blockInfo, error);
    }

    bool SetReferences::initializeInitialConditions(BlockInformation */*blockInfo*/, wbt::Error */*error*/)
    {
        //Simply reset the variable m_resetControlMode
        //It will be read at the first cycle of output
        m_resetControlMode = true;
        return true;
    }

    bool SetReferences::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        //get input
        wbi::wholeBodyInterface * const interface = WBInterface::sharedInstance().interface();
        if (!interface) return false;

        Signal references = blockInfo->getInputPortSignal(0);
        for (unsigned i = 0; i < blockInfo->getInputPortWidth(0); ++i) {
            m_references[i] = references.get(i).doubleData();
        }

        if (m_resetControlMode) {
            m_resetControlMode = false;

            //now switch control mode
            if (m_fullControl) {
                interface->setControlMode(m_controlMode, m_references);
            } else {
                for (int i = 0; i < m_controlledJoints.size(); i++) {
                    interface->setControlMode(m_controlMode, &m_references[i], m_controlledJoints[i]);
                }
            }
            return true;
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
}
