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

class wbt::YarpWrite : public wbt::Block {
public:
    static std::string ClassName;
    
    YarpWrite();
    
    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool initialize(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool terminate(BlockInformation *blockInfo, wbt::Error *error);
    virtual bool output(BlockInformation *blockInfo, wbt::Error *error);
    
private:
    bool m_autoconnect;
    bool m_errorOnMissingPort;

    std::string m_destinationPortName;
    yarp::os::BufferedPort<yarp::sig::Vector> *m_port;
};

#endif /* end of include guard: WBT_YARPWRITE_H */
