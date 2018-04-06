/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef WBT_LOG_H
#define WBT_LOG_H

#include <sstream>
#include <string>
#include <vector>

#ifdef NDEBUG
#define WBT_LOG_VERBOSITY wbt::Log::RELEASE
#else
#define WBT_LOG_VERBOSITY wbt::Log::DEBUG
#endif

#ifndef wbtError
#define wbtError \
    wbt::Log::getSingleton().getLogStringStream(wbt::Log::ERROR, __FILE__, __LINE__, __FUNCTION__)
#endif

#ifndef wbtWarning
#define wbtWarning \
    wbt::Log::getSingleton().getLogStringStream(wbt::Log::WARNING, __FILE__, __LINE__, __FUNCTION__)
#endif

namespace wbt {
    class Log;
} // namespace wbt

/**
 * Basic Log class
 */
class wbt::Log
{
public:
    enum LogType
    {
        ERROR,
        WARNING
    };

    enum LogVerbosity
    {
        RELEASE,
        DEBUG
    };

private:
    std::vector<std::stringstream> m_errorsSStream;
    std::vector<std::stringstream> m_warningsSStream;

    const LogVerbosity m_verbosity = WBT_LOG_VERBOSITY;

    std::string serializeVectorStringStream(const std::vector<std::stringstream>& ss) const;

public:
    Log() = default;
    ~Log() = default;

    static wbt::Log& getSingleton();

    std::stringstream& getLogStringStream(const LogType& type,
                                          const std::string& file,
                                          const unsigned& line,
                                          const std::string& function);

    std::string getErrors() const;
    std::string getWarnings() const;

    void clearErrors();
    void clearWarnings();

    void clear();
};

#endif // WBT_LOG_H
