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
 * @brief Class for handling log messages
 *
 * Errors and Warnings are currently supported.
 */
class wbt::Log
{
public:
    enum LogType
    {
        ERROR,
        WARNING
    };

private:
    /**
     * @brief Define the verbosity of the logs
     *
     * The verbosity is changed automatically detecting if the class is compiled in Debug.
     */
    enum LogVerbosity
    {
        RELEASE,
        DEBUG
    };

    std::vector<std::stringstream> m_errorsSStream;
    std::vector<std::stringstream> m_warningsSStream;

    const LogVerbosity m_verbosity = WBT_LOG_VERBOSITY;

    std::string serializeVectorStringStream(const std::vector<std::stringstream>& ss) const;

public:
    Log() = default;
    ~Log() = default;

    /**
     * @brief Get the Log singleton
     *
     * There is only one instance in the whole program of this class.
     *
     * @return The log singleton.
     */
    static wbt::Log& getSingleton();

    /**
     * @brief Get the stringstream object for adding log messages
     *
     * @param type The log type.
     * @param file The file from which this method is called (preprocessor directive).
     * @param line The line from which this method is called (preprocessor directive).
     * @param function The function from which this method is called (preprocessor directive).
     * @return The stringstream object matching the log type.
     */
    std::stringstream& getLogStringStream(const LogType& type,
                                          const std::string& file,
                                          const unsigned& line,
                                          const std::string& function);

    /**
     * @brief Get the stored error messages.
     * @return The error messages.
     */
    std::string getErrors() const;

    /**
     * @brief Get the stored warning messages.
     * @return The warning messages.
     */
    std::string getWarnings() const;

    /**
     * @brief Clear the stored error messages.
     */
    void clearErrors();

    /**
     * @brief Clear the stored warning messages.
     */
    void clearWarnings();

    /**
     * @brief Clear all the stored log messages.
     */
    void clear();
};

#endif // WBT_LOG_H
