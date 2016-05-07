#include "SetLowLevelPID.h"

#include "Error.h"
#include "WBInterface.h"
#include <yarpWholeBodyInterface/yarpWholeBodyInterface.h>
#include <yarpWholeBodyInterface/yarpWbiUtil.h>
#include <wbi/wholeBodyInterface.h>
#include <codyco/PIDList.h>
#include <yarp/os/ResourceFinder.h>
#include <map>

namespace wbt {

    static const std::string TorquePIDInitialKey = "__ORIGINAL_PIDs__";
    static const std::string TorquePIDDefaultKey = "__DEFAULT_PIDs__";

#pragma mark - SetLowLevelPID class implementation

    std::string SetLowLevelPID::ClassName = "SetLowLevelPID";

    SetLowLevelPID::SetLowLevelPID()
    : m_firstRun(true)
    , m_lastGainSetIndex(-1)
    , m_controlMode(wbi::CTRL_MODE_TORQUE) {}


    bool SetLowLevelPID::loadLowLevelGainsFromFile(std::string filename,
                                                   const codyco::PIDList &originalList,
                                                   wbi::wholeBodyInterface& interface,
                                                   codyco::PIDList &loadedPIDs)
    {
        //List of list. Each element has a key: joint name, and a list of pairs: kp, kd, ki and its respective value
        using namespace yarp::os;
        yarp::os::ResourceFinder resourceFinder = yarp::os::ResourceFinder::getResourceFinderSingleton();
        Property file;
        std::string fileName = resourceFinder.findFileByName(filename);
        if (fileName.empty()) return false;
        file.fromConfigFile(fileName);

        Bottle externalList;
        externalList.fromString(file.toString());

        bool result = true;
        wbi::IDList jointList = interface.getJointList();
        for (int i = 0; i < externalList.size(); ++i) {
            if (!externalList.get(i).isList()) continue;
            Bottle *jointConfig = externalList.get(i).asList();
            if (jointConfig->size() < 2 || !jointConfig->get(0).isString()) continue;
            wbi::ID jointID(jointConfig->get(0).asString());
            int jointIndex = -1;
            if (!jointList.idToIndex(jointID, jointIndex)) continue;
            if (jointIndex < 0 || jointIndex >= jointList.size()) {
                yWarning("Specified joint %s index is outside joint list size", jointID.toString().c_str());
                continue;
            }

            loadedPIDs.pidList()[jointIndex] = originalList.pidList()[jointIndex];
            loadedPIDs.motorParametersList()[jointIndex] = originalList.motorParametersList()[jointIndex];

            result = result && yarpWbi::pidFromBottleDescription(*jointConfig, loadedPIDs.pidList()[jointIndex]);
            if (m_controlMode == wbi::CTRL_MODE_TORQUE) {
                result = result && yarpWbi::motorTorqueParametersFromBottleDescription(*jointConfig, loadedPIDs.motorParametersList()[jointIndex]);
            }
        }
        return result;
    }

    bool SetLowLevelPID::loadGainsFromValue(const yarp::os::Value &gains,
                                            PidMap &pidMap,
                                            wbi::wholeBodyInterface& interface)
    {
        pidMap.clear();

        yarpWbi::yarpWholeBodyInterface *yarpInterface = dynamic_cast<yarpWbi::yarpWholeBodyInterface *>(&interface);
        if (!yarpInterface) {
            return false;
        }

        //Load original gains from controlboards and save them to the original key.
        codyco::PIDList originalGains(interface.getDoFs());
        yarpWbi::yarpWholeBodyActuators *actuators = yarpInterface->wholeBodyActuator();
        if (!actuators) {
            return false;
        }
        //TODO: should be made not limited to torque
        actuators->getPIDGains(originalGains.pidList(), wbi::CTRL_MODE_TORQUE);
        actuators->getMotorTorqueParameters(originalGains.motorParametersList());
        pidMap.insert(PidMap::value_type(TorquePIDInitialKey, originalGains));

        //Now load additional gains
        bool result = true;
        if (gains.isString()) {
            codyco::PIDList pids(interface.getDoFs());
            result = loadLowLevelGainsFromFile(gains.asString(), originalGains, interface, pids);
            pidMap.insert(PidMap::value_type(TorquePIDDefaultKey, pids));
        } else if (gains.isList()) {
            using namespace yarp::os;
            Bottle *list = gains.asList();

            //list of files. gains will be saved as integer-values
            for (int i = 0; i < list->size(); ++i) {
                if (!list->get(i).isString()) continue;
                std::string filename = list->get(i).asString();
                codyco::PIDList pids(interface.getDoFs());
                result = loadLowLevelGainsFromFile(filename, originalGains, interface, pids);

                if (result) {
                    pidMap.insert(PidMap::value_type(yarpWbi::stringFromInt(i + 1), pids));
                }
            }
        }
        return result;
    }

    //TODO: for performance it is probably better to change the map so that the index is an integer
    //i.e. a non continuous vector
    bool SetLowLevelPID::setCurrentGains(const PidMap &pidMap,
                                         std::string key,
                                         wbi::iWholeBodyActuators& actuators)
    {
        yarpWbi::yarpWholeBodyActuators *yarpActuators = static_cast<yarpWbi::yarpWholeBodyActuators *>(&actuators);
        if (!yarpActuators) return false;

        PidMap::const_iterator found = pidMap.find(key);
        //Treat one exception: pidMap with size == 2, the default can be set to either the string or the num 1
        if (found == pidMap.end() && key == TorquePIDDefaultKey && pidMap.size() == 2) {
            found = pidMap.find("1");
        }

        if (found == pidMap.end()) return false;
        bool result = yarpActuators->setPIDGains(found->second.pidList(), m_controlMode); //to be made generic (torque, position, etc)
        if (m_controlMode == wbi::CTRL_MODE_TORQUE) {
            result = result && yarpActuators->setMotorTorqueParameters(found->second.motorParametersList());
        }
        return result;
    }

#pragma mark - overloaded methods

    unsigned SetLowLevelPID::numberOfParameters()
    {
        return WBIBlock::numberOfParameters()
        + 1 // pid parameters file
        + 1;// control method
    }

    bool SetLowLevelPID::configureSizeAndPorts(SimStruct *S, wbt::Error *error)
    {
        if (!WBIBlock::configureSizeAndPorts(S, error)) {
            return false;
        }

        // Specify I/O
        if (!ssSetNumInputPorts (S, 0)) {
            if (error) error->message = "Failed to configure the number of input ports";
            return false;
        }

        // Output port:
        if (!ssSetNumOutputPorts (S, 0)) {
            if (error) error->message = "Failed to configure the number of output ports";
            return false;
        }

        return true;
    }

    bool SetLowLevelPID::initialize(SimStruct *S, wbt::Error *error)
    {
        using namespace yarp::os;
        if (!WBIBlock::initialize(S, error)) return false;

        // Reading the control type
        std::string controlType;
        if (!Block::readStringParameterAtIndex(S, WBIBlock::numberOfParameters() + 2, controlType)) {
            if (error) error->message = "Could not read control type parameter";
            return false;
        }

        m_controlMode = wbi::CTRL_MODE_UNKNOWN;
        if (controlType == "Position") {
            m_controlMode = wbi::CTRL_MODE_POS;
        } else if (controlType == "Torque") {
            m_controlMode = wbi::CTRL_MODE_TORQUE;
        } else {
            if (error) error->message = "Control Mode not supported";
            return false;
        }


        // Reading the PID specification parameter
        std::string pidParameter;
        if (!Block::readStringParameterAtIndex(S, WBIBlock::numberOfParameters() + 1, pidParameter)) {
            if (error) error->message = "Could not read PID file specification parameter";
            return false;
        }

        Value value;
        value.fromString(pidParameter.c_str());
        m_pids.clear();

        yarpWbi::yarpWholeBodyInterface * const interface = dynamic_cast<yarpWbi::yarpWholeBodyInterface * const>(WBInterface::sharedInstance().interface());
        if (!interface) {
            if (error) error->message = "This block currently work only with YARP-WBI implementation";
            return false;
        }
        if (!loadGainsFromValue(value, m_pids, *interface)) {
            m_pids.clear();
            if (error) error->message = "Error while loading PIDs configuration";
            yError("Error while loading PIDs configuration");
            return false;
        } else {
            yInfo("Loaded PIDs configuration");
        }
        m_lastGainSetIndex = -1;

        m_firstRun = true;
        return true;
    }

    bool SetLowLevelPID::terminate(SimStruct *S, wbt::Error *error)
    {
        //static_cast as the dynamic has been done in the initialize
        //and the pointer should not change
        yarpWbi::yarpWholeBodyInterface * const interface = static_cast<yarpWbi::yarpWholeBodyInterface * const>(WBInterface::sharedInstance().interface());

        if (interface) {
            yarpWbi::yarpWholeBodyActuators *actuators = interface->wholeBodyActuator();
            if (actuators && m_pids.size() > 1) {
                setCurrentGains(m_pids, TorquePIDInitialKey, *actuators);
            }
        }

        m_pids.clear();
        m_lastGainSetIndex = -1;

        return WBIBlock::terminate(S, error);
    }

    bool SetLowLevelPID::output(SimStruct *S, wbt::Error *error)
    {
        //static_cast as the dynamic has been done in the initialize
        //and the pointer should not change
        yarpWbi::yarpWholeBodyInterface * const interface = static_cast<yarpWbi::yarpWholeBodyInterface * const>(WBInterface::sharedInstance().interface());
        if (interface) {
            if (m_firstRun) {
                m_firstRun = false;

                yarpWbi::yarpWholeBodyActuators *actuators = interface->wholeBodyActuator();
                if (!actuators) {
                    if (error) error->message = "Failed to retrieve the interface to the actuators";
                    return false;
                }

                //First case: only one element
                if (m_lastGainSetIndex == -1 && m_pids.size() == 2) {
                    //just switch to the only existing set
                    setCurrentGains(m_pids, TorquePIDDefaultKey, *actuators);
                    m_lastGainSetIndex = 0;
                } else {
//                    InputPtrsType      u     = ssGetInputPortSignalPtrs(S, 0);
//                    InputInt8PtrsType  uPtrs = (InputInt8PtrsType)u;
//                    if (*uPtrs[0] != lastGainIndex) {
//                        wbitoolbox::setCurrentGains(*pids, *uPtrs[0], *((yarpWbi::yarpWholeBodyInterface*)robot->wbInterface));
//                        info[0] = *uPtrs[0];
//                    }
                }

            }
            return true;
        }
        return false;
    }
}
