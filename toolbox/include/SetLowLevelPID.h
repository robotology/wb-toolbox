#ifndef WBT_SETLOWLEVELPID_H_
#define WBT_SETLOWLEVELPID_H_

#include "WBIBlock.h"
#include <wbi/wbiConstants.h>
#include <map>

namespace wbt {
    class SetLowLevelPID;
}

namespace codyco {
    class PIDList;
}

namespace wbi {
    class wholeBodyInterface;
    class iWholeBodyActuators;
}

namespace yarp {
    namespace os {
        class Value;
    }
}

typedef std::map<std::string, codyco::PIDList> PidMap;

class wbt::SetLowLevelPID : public wbt::WBIBlock {

    bool m_firstRun;
    PidMap m_pids;
    int m_lastGainSetIndex;
    wbi::ControlMode m_controlMode;

    bool loadLowLevelGainsFromFile(std::string filename,
                                   const codyco::PIDList &originalList,
                                   wbi::wholeBodyInterface& interface,
                                   codyco::PIDList &loadedPIDs);

    bool loadGainsFromValue(const yarp::os::Value &gains,
                            PidMap &pidMap,
                            wbi::wholeBodyInterface& interface);

    bool setCurrentGains(const PidMap &pidMap,
                         std::string key,
                         wbi::iWholeBodyActuators& actuators);

public:
    static std::string ClassName;
    SetLowLevelPID();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);

    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);

    
};



#endif /* end of include guard: WBT_SETLOWLEVELPID_H_ */
