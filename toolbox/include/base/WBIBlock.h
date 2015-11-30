#ifndef WBIT_WBIBLOCK_H
#define WBIT_WBIBLOCK_H

#include "Block.h"

namespace wbit {
    class WBIBlock;
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
class wbit::WBIBlock : public wbit::Block {

public:
    virtual ~WBIBlock();
    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbit::Error *error);
    virtual bool initialize(SimStruct *S, wbit::Error *error);
    virtual bool terminate(SimStruct *S, wbit::Error *error);
};


#endif /* end of include guard: WBIT_WBIBLOCK_H */
