#ifndef WBT_MODELPARTITIONER_H
#define WBT_MODELPARTITIONER_H

#include "WBBlock.h"
#include "RobotInterface.h"
#include <vector>
#include <memory>

namespace wbt {
    class ModelPartitioner;
}

class wbt::ModelPartitioner : public wbt::WBBlock
{
private:
    bool m_yarp2WBI;

    std::shared_ptr<JointsMapString> m_jointsMapString;

public:
    static const std::string ClassName;
    ModelPartitioner() = default;
    ~ModelPartitioner() override = default;

    unsigned numberOfParameters() override;

    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif /* end of include guard: WBT_MODELPARTITIONER_H */
