#ifndef WBT_YARPWRITE_H
#define WBT_YARPWRITE_H

#include "Block.h"
#include <memory>

namespace wbt {
    class YarpWrite;
}

namespace yarp {
    namespace os {
        template <class T>
        class BufferedPort;
    }
    namespace sig {
        class Vector;
    }
}

class wbt::YarpWrite : public wbt::Block
{
public:
    static const std::string ClassName;

    YarpWrite();
    ~YarpWrite() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;

private:
    bool m_autoconnect;
    bool m_errorOnMissingPort;

    std::string m_destinationPortName;
    std::unique_ptr<yarp::os::BufferedPort<yarp::sig::Vector>> m_port;
};

#endif /* end of include guard: WBT_YARPWRITE_H */
