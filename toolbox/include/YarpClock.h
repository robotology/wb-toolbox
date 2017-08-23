#ifndef WBT_YARPCLOCK_H
#define WBT_YARPCLOCK_H

#include "Block.h"

namespace wbt {
    class YarpClock;
}

class wbt::YarpClock : public wbt::Block {
public:
    static std::string ClassName;

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);

};

#endif /* end of include guard: WBT_YARPCLOCK_H */
