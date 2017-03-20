#ifndef WBT_SETREFERENCES_H
#define WBT_SETREFERENCES_H

#include "WBIBlock.h"
#include <wbi/wbiConstants.h>
#include <vector>

namespace wbt {
    class SetReferences;
}

class wbt::SetReferences : public wbt::WBIBlock {

    double *m_references;
    wbi::ControlMode m_controlMode;
    bool m_fullControl;
    std::vector<int> m_controlledJoints;
    bool m_resetControlMode;

public:
    static std::string ClassName;
    SetReferences();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);

    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool initializeInitialConditions(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);


};


#endif /* end of include guard: WBT_SETREFERENCES_H */
