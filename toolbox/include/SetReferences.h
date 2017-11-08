#ifndef WBT_SETREFERENCES_H
#define WBT_SETREFERENCES_H

#include "WBBlock.h"
#include <vector>

namespace wbt {
    class SetReferences;
}

class wbt::SetReferences : public wbt::WBBlock
{
private:
    std::vector<int> m_controlModes;
    bool m_resetControlMode;
    static void rad2deg(std::vector<double>& v);

public:
    static const std::string ClassName;

    SetReferences();

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(BlockInformation* blockInfo) override;
    bool initializeInitialConditions(BlockInformation* blockInfo) override;
    bool terminate(BlockInformation* blockInfo) override;
    bool output(BlockInformation* blockInfo) override;
};

#endif /* end of include guard: WBT_SETREFERENCES_H */
