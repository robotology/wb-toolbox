/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

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

/**
 * @brief The wbt::YarpRead class
 *
 * In addition to @ref block_parameters, wbt::YarpRead requires:
 *
 * | Type | Index | Rows  | Cols  | Name  |
 * | ---- | :---: | :---: | :---: | ----- |
 * | ::STRING | 0 + Block::NumberOfParameters | 1 | 1 | "PortName"           |
 * | ::INT    | 1 + Block::NumberOfParameters | 1 | 1 | "SignalSize"         |
 * | ::BOOL   | 2 + Block::NumberOfParameters | 1 | 1 | "WaitData"           |
 * | ::BOOL   | 3 + Block::NumberOfParameters | 1 | 1 | "ReadTimestamp"      |
 * | ::BOOL   | 4 + Block::NumberOfParameters | 1 | 1 | "Autoconnect"        |
 * | ::DOUBLE | 5 + Block::NumberOfParameters | 1 | 1 | "Timeout"            |
 * | ::BOOL   | 6 + Block::NumberOfParameters | 1 | 1 | "ErrorOnMissingPort" |
 *
 */
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
