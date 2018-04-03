#ifndef WBT_SETLOWLEVELPID_H_
#define WBT_SETLOWLEVELPID_H_

#include "WBBlock.h"
#include <unordered_map>
#include <yarp/dev/IPidControl.h>

namespace wbt {
    class SetLowLevelPID;
    typedef std::tuple<double, double, double> PidData;
} // namespace wbt

namespace yarp {
    namespace dev {
        class Pid;
    }
} // namespace yarp

class wbt::SetLowLevelPID : public wbt::WBBlock
{
private:
    std::vector<yarp::dev::Pid> m_appliedPidValues;
    std::vector<yarp::dev::Pid> m_defaultPidValues;
    std::unordered_map<std::string, PidData> m_pidJointsFromParameters;

    yarp::dev::PidControlTypeEnum m_controlType;

    bool readWBTPidConfigObject(const BlockInformation* blockInfo);

public:
    static const std::string ClassName;

    SetLowLevelPID() = default;
    ~SetLowLevelPID() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* end of include guard: WBT_SETLOWLEVELPID_H_ */
