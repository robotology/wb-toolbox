#include "Log.h"
#include <sstream>
#include <iterator>

using namespace wbt;

Log& Log::getSingleton()
{
    static Log logInstance;
    return logInstance;
}

void Log::error(std::string errorMessage)
{
    errors.push_back(errorMessage);
}

void Log::warning(std::string warningMessage)
{
    warnings.push_back(warningMessage);
}

void Log::setPrefix(std::string prefixMessage)
{
    prefix = prefixMessage;
}

void Log::resetPrefix()
{
    prefix.clear();
}

std::string Log::serializeVectorString(std::vector<std::string> v, std::string prefix)
{
    std::stringstream output;
    std::ostream_iterator<std::string> output_iterator(output, "\n");
    std::copy(v.begin(), v.end(), output_iterator);

    if (prefix.empty()) {
        return output.str();
    }
    else {
        return prefix + "\n" + output.str();
    }
}

std::string Log::getErrors() const
{
    return serializeVectorString(errors, prefix);
}

std::string Log::getWarnings() const
{
    return serializeVectorString(warnings, prefix);
}

void Log::clearWarnings()
{
    warnings.clear();
}

void Log::clearErrors()
{
    errors.clear();
}

void Log::clear()
{
    clearErrors();
    clearWarnings();
}
