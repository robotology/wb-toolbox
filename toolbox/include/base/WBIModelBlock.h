#ifndef WBT_WBIMODELBLOCK_H
#define WBT_WBIMODELBLOCK_H

#include "WBIBlock.h"

namespace wbt {
    class WBIModelBlock;
    class BlockInformation;
}

/**
 * Basic class for WBI related blocks.
 * This class (the whole toolbox in reality) assumes the block represent
 * an instantaneous system (i.e. not a dynamic system).
 *
 * You can create a new block by deriving this class and implementing at least
 * the output method.
 *
 * This block implements the following default behaviours:
 * - it ask for 4 parameters (robot name, local (module) name, wbi config file and wbi list)
 * - It initializes the yarp network and the whole body interface object
 * - During terminate it closes and release the interface object and terminate the yarp network
 *
 * @Note: Usually you want to call this class implementations at some point in your
 * method overridings, unless you want to completely change the code (but at that point
 * you probabily want to derive from Block instead)
 */
class wbt::WBIModelBlock : public wbt::WBIBlock {

public:
    virtual ~WBIModelBlock();
    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
};

#endif /* WBT_WBIMODELBLOCK_H */
