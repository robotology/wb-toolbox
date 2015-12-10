#ifndef WBT_YARPREAD_H
#define WBT_YARPREAD_H

#include "Block.h"

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

class wbt::YarpRead : public wbt::Block {
public:
    static std::string ClassName;
    
    YarpRead();
    
    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbt::Error *error);
    virtual bool initialize(SimStruct *S, wbt::Error *error);
    virtual bool terminate(SimStruct *S, wbt::Error *error);
    virtual bool output(SimStruct *S, wbt::Error *error);
    
private:
    bool m_autoconnect;
    bool m_blocking;
    bool m_shouldReadTimestamp;
    bool m_errorOnMissingPort;
    
    yarp::os::BufferedPort<yarp::sig::Vector> *m_port;
};

#endif /* end of include guard: WBT_YARPREAD_H */
