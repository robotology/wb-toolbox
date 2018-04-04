#ifndef WBT_YARPREAD_H
#define WBT_YARPREAD_H

#include "Block.h"
#include <memory>

namespace wbt {
    class YarpRead;
}

namespace yarp {
    namespace os {
        template <class T>
        class BufferedPort;
    }
    namespace sig {
        class Vector;
    }
} // namespace yarp

class wbt::YarpRead : public wbt::Block
{
private:
    bool m_autoconnect = false;
    bool m_blocking = false;
    bool m_shouldReadTimestamp = false;
    bool m_errorOnMissingPort = true;
    int m_bufferSize;
    double m_timeout = 1.0;

    std::unique_ptr<yarp::os::BufferedPort<yarp::sig::Vector>> m_port;

public:
    static const std::string ClassName;

    YarpRead() = default;
    ~YarpRead() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_YARPREAD_H
