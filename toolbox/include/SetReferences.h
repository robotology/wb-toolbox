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
    static const std::vector<double> rad2deg(const double* buffer, const unsigned width);

public:
    static const std::string ClassName;

    SetReferences();
    ~SetReferences() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(const BlockInformation* blockInfo) override;
    bool initializeInitialConditions(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* end of include guard: WBT_SETREFERENCES_H */
