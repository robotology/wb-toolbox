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
public:
    static const std::string ClassName;

    YarpRead();
    ~YarpRead() override = default;

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(const BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;

private:
    bool m_autoconnect;
    bool m_blocking;
    bool m_shouldReadTimestamp;
    bool m_errorOnMissingPort;
    int m_bufferSize;
    double m_timeout;

    std::unique_ptr<yarp::os::BufferedPort<yarp::sig::Vector>> m_port;
};

#endif /* end of include guard: WBT_YARPREAD_H */
