#ifndef WBT_ERROR_H
#define WBT_ERROR_H

#include <string>

namespace wbt {
    class Error;
}

/**
 * Basic Error class
 */
class wbt::Error {
public:
    std::string message; /*<! a string representing the error message */
};


#endif /* end of include guard: WBT_ERROR_H */
