/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_YARPWRITE_H
#define WBT_YARPWRITE_H

#include "Block.h"
#include <memory>
#include <yarp/sig/Vector.h>

namespace wbt {
    class YarpWrite;
}

namespace yarp {
    namespace os {
        template <class T>
        class BufferedPort;
    }
} // namespace yarp

class wbt::YarpWrite : public wbt::Block
{
private:
    bool m_autoconnect = false;
    bool m_errorOnMissingPort = true;

    std::string m_destinationPortName;
    yarp::sig::Vector m_outputVector;
    std::unique_ptr<yarp::os::BufferedPort<yarp::sig::Vector>> m_port;

public:
    static const std::string ClassName;

    YarpWrite() = default;
    ~YarpWrite() override = default;

    unsigned numberOfParameters() override;
    bool parseParameters(BlockInformation* blockInfo) override;
    bool configureSizeAndPorts(BlockInformation* blockInfo) override;
    bool initialize(BlockInformation* blockInfo) override;
    bool terminate(const BlockInformation* blockInfo) override;
    bool output(const BlockInformation* blockInfo) override;
};

#endif // WBT_YARPWRITE_H
