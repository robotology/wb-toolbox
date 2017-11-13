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
}

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

    std::unique_ptr<yarp::os::BufferedPort<yarp::sig::Vector>> m_port;

    static const unsigned PARAM_IDX_PORTNAME; // port name
    static const unsigned PARAM_IDX_PORTSIZE; // Size of the port you're reading
    static const unsigned PARAM_IDX_WAITDATA; // boolean for blocking reading
    static const unsigned PARAM_IDX_READ_TS;  // boolean to stream timestamp
    static const unsigned PARAM_IDX_AUTOCONNECT; // Autoconnect boolean
    static const unsigned PARAM_IDX_ERR_NO_PORT; // Error on missing port if autoconnect is on boolean
};

#endif /* end of include guard: WBT_YARPREAD_H */
