#ifndef WBT_SETLOWLEVELPID_H_
#define WBT_SETLOWLEVELPID_H_

#include "WBIBlock.h"
#include <wbi/wbiConstants.h>
#include <map>

namespace wbt {
    class SetLowLevelPID;
}

namespace wbi {
    class wholeBodyInterface;
    class iWholeBodyActuators;
}

namespace yarpWbi {
    class PIDList;
}

namespace yarp {
    namespace os {
        class Value;
    }
}

typedef std::map<std::string, yarpWbi::PIDList> PidMap;

class wbt::SetLowLevelPID : public wbt::WBIBlock {

    bool m_firstRun;
    PidMap m_pids;
    int m_lastGainSetIndex;
    wbi::ControlMode m_controlMode;

    bool loadLowLevelGainsFromFile(std::string filename,
                                   const yarpWbi::PIDList &originalList,
                                   wbi::wholeBodyInterface& interface,
                                   yarpWbi::PIDList &loadedPIDs);

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
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);

    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);

    
};



#endif /* end of include guard: WBT_SETLOWLEVELPID_H_ */
