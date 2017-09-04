#ifndef WBT_GETCONTROLREFERENCES_H
#define WBT_GETCONTROLREFERENCES_H

#include "WBIBlock.h"
#include <wbi/wbiConstants.h>
#include <vector>

namespace wbt {
    class GetControlReferences;
}

class wbt::GetControlReferences : public wbt::WBIBlock {

    double *m_currentControlReferences;
    wbi::ControlMode m_controlMode;
    bool m_fullControl;
    std::vector<int> m_controlledJoints;

public:
    static std::string ClassName;
    GetControlReferences();

    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);

    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);


};


#endif /* end of include guard: WBT_GETCONTROLREFERENCES_H */
