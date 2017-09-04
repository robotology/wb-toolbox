#include "GetControlReferences.h"

#include "Error.h"
#include "WBInterface.h"
#include "BlockInformation.h"
#include "Signal.h"
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <yarpWholeBodyInterface/yarpWholeBodyActuators.h>
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <wbi/wholeBodyInterface.h>

namespace wbt {

    std::string GetControlReferences::ClassName = "GetControlReferences";

    GetControlReferences::GetControlReferences()
    : m_currentControlReferences(0)
    , m_controlMode(wbi::CTRL_MODE_UNKNOWN)
    , m_fullControl(true)
    {}

    unsigned GetControlReferences::numberOfParameters()
    {
        // 1 - Control Type
        // 2 - Full/Sublist type
        // 3 - (only if sublist) controlled joints
        return WBIBlock::numberOfParameters() + 3;
    }

    bool GetControlReferences::configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error)
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
        if (!blockInfo->setNumberOfInputPorts(0)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }

        // Output port:
        if (!blockInfo->setNumberOfOuputPorts(1)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        bool success = blockInfo->setOutputPortVectorSize(0, dofs);
        blockInfo->setOutputPortType(0, PortDataTypeDouble);

        if (!success) {
            if (error) error->message = "Failed to configure output ports";
            return false;
        }

        return true;
    }

    bool GetControlReferences::initialize(BlockInformation *blockInfo, wbt::Error *error)
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
        m_currentControlReferences = new double[dofs];

        return m_currentControlReferences;
    }

    bool GetControlReferences::terminate(BlockInformation *blockInfo, wbt::Error *error)
    {
        if (m_currentControlReferences) {
            delete [] m_currentControlReferences;
            m_currentControlReferences = 0;
        }
        return WBIBlock::terminate(blockInfo, error);
    }

    bool GetControlReferences::output(BlockInformation *blockInfo, wbt::Error */*error*/)
    {
        // This is a temporary workaround waiting for toolbox refactoring
        yarpWbi::yarpWholeBodyInterface* interface = static_cast<yarpWbi::yarpWholeBodyInterface*>(WBInterface::sharedInstance().interface());
        if (!interface) return false;

        yarpWbi::yarpWholeBodyActuators* actuatorsInterface = interface->wholeBodyActuator();
        if (!actuatorsInterface) return false;

        if (m_fullControl) {
            actuatorsInterface->getControlReferences(m_controlMode, m_currentControlReferences);
        } else {
            for (int i = 0; i < m_controlledJoints.size(); i++) {
                actuatorsInterface->getControlReferences(m_controlMode, &m_currentControlReferences[i], m_controlledJoints[i]);
            }
        }

        Signal signal = blockInfo->getOutputPortSignal(0);
        signal.setBuffer(m_currentControlReferences, blockInfo->getOutputPortWidth(0));

        return true;
    }
}
