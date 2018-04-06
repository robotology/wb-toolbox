/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include "Log.h"

#include <iterator>
#include <sstream>

using namespace wbt;

Log& Log::getSingleton()
{
    static Log logInstance;
    return logInstance;
}

std::stringstream& Log::getLogStringStream(const Log::LogType& type,
                                           const std::string& file,
                                           const unsigned& line,
                                           const std::string& function)
{
    switch (m_verbosity) {
        case RELEASE:
            switch (type) {
                case ERROR:
                    m_errorsSStream.emplace_back();
                    return m_errorsSStream.back();
                case WARNING:
                    m_warningsSStream.emplace_back();
                    return m_warningsSStream.back();
            }
        case DEBUG:
            switch (type) {
                case ERROR: {
                    m_errorsSStream.emplace_back();
                    auto& ss = m_errorsSStream.back();
                    ss << std::endl
                       << file << "@" << function << ":" << std::to_string(line) << std::endl;
                    return ss;
                }
                case WARNING: {
                    m_warningsSStream.emplace_back();
                    auto& ss = m_warningsSStream.back();
                    ss << std::endl
                       << file << "@" << function << ":" << std::to_string(line) << std::endl;
                    return ss;
                }
            }
    }
}

std::string Log::serializeVectorStringStream(const std::vector<std::stringstream>& ss) const
{
    std::stringstream output;

    for (const auto& ss_elem : ss) {
        output << ss_elem.str() << std::endl;
    }

    return output.str();
}

std::string Log::getErrors() const
{
    return serializeVectorStringStream(m_errorsSStream);
}

std::string Log::getWarnings() const
{
    return serializeVectorStringStream(m_warningsSStream);
}

void Log::clearWarnings()
{
    m_warningsSStream.clear();
}

void Log::clearErrors()
{
    m_errorsSStream.clear();
}

void Log::clear()
{
    clearErrors();
    clearWarnings();
}
