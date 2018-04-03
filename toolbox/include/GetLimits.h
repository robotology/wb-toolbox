#ifndef WBT_GETLIMITS_H
#define WBT_GETLIMITS_H

#include "WBBlock.h"
#include <memory>

namespace wbt {
    class GetLimits;
    struct Limit;
} // namespace wbt

struct wbt::Limit
{
    std::vector<double> min;
    std::vector<double> max;

    Limit(unsigned size = 0)
        : min(size)
        , max(size)
    {}
};

class wbt::GetLimits : public wbt::WBBlock
{
private:
    std::unique_ptr<Limit> m_limits;
    static double deg2rad(const double& v);

public:
    static const std::string ClassName;

    GetLimits() = default;
    ~GetLimits() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;

    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* WBT_GETLIMITS_H */
