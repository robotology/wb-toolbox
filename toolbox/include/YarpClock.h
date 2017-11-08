#ifndef WBT_YARPCLOCK_H
#define WBT_YARPCLOCK_H

#include "Block.h"

namespace wbt {
    class YarpClock;
}

class wbt::YarpClock : public wbt::Block
{
public:
    static const std::string ClassName;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(BlockInformation* blockInfo) override;
    bool output(BlockInformation* blockInfo) override;
};

#endif /* end of include guard: WBT_YARPCLOCK_H */
