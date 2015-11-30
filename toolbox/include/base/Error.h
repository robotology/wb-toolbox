#ifndef WBIT_ERROR_H
#define WBIT_ERROR_H

#include <string>

namespace wbit {
    class Error;
}

/**
 * Basic Error class
 */
class wbit::Error {
public:
    std::string message; /*<! a string representing the error message */
};


#endif /* end of include guard: WBIT_ERROR_H */
