#ifndef WBT_LOG_H
#define WBT_LOG_H

#include <string>
#include <vector>

namespace wbt {
    class Log;
}

/**
 * Basic Log class
 */
class wbt::Log
{
private:
    std::vector<std::string> errors;
    std::vector<std::string> warnings;
    std::string prefix;

    static std::string serializeVectorString(std::vector<std::string> v, std::string prefix="");
public:

    static wbt::Log& getSingleton();

    void error(std::string errorMessage);
    void warning(std::string warningMessage);

    void resetPrefix();
    void setPrefix(std::string prefixMessage);

    std::string getErrors() const;
    std::string getWarnings() const;

    void clearErrors();
    void clearWarnings();

    void clear();
};


#endif /* end of include guard: WBT_LOG_H */
