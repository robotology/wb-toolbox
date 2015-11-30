#include "Block.h"

#ifndef WBIT_YARPREAD_H
#define WBIT_YARPREAD_H

namespace wbit {
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

class wbit::YarpRead : public wbit::Block {
public:
    static std::string ClassName;
    
    YarpRead();
    virtual ~YarpRead();
    
    virtual unsigned numberOfParameters();
    virtual bool configureSizeAndPorts(SimStruct *S, wbit::Error *error);
    virtual bool initialize(SimStruct *S, wbit::Error *error);
    virtual bool terminate(SimStruct *S, wbit::Error *error);
    virtual bool output(SimStruct *S, wbit::Error *error);
    
private:
    int_T m_autoconnect;
    int_T m_blocking;
    int_T m_shouldReadTimestamp;
    int_T m_errorOnMissingPort;
    
    yarp::os::BufferedPort<yarp::sig::Vector> *m_port;
};

#endif /* end of include guard: WBIT_YARPREAD_H */
