#ifndef WBT_YARPWRITE_H
#define WBT_YARPWRITE_H

#include "Block.h"

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

    unsigned numberOfParameters() override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(BlockInformation* blockInfo) override;
    bool output(BlockInformation* blockInfo) override;

private:
    bool m_autoconnect;
    bool m_errorOnMissingPort;

    std::string m_destinationPortName;
    yarp::os::BufferedPort<yarp::sig::Vector>* m_port;

    static const unsigned PARAM_IDX_PORTNAME;    // Port name
    static const unsigned PARAM_IDX_AUTOCONNECT; // Autoconnect boolean
    static const unsigned PARAM_IDX_ERR_NO_PORT; // Error on missing port if autoconnect is true
};

#endif /* end of include guard: WBT_YARPWRITE_H */
