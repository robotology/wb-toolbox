#ifndef WBT_DEBUG_H
#define WBT_DEBUG_H

// Debug messages to display in the console from which Matlab was launched
// NDEBUG is defined by CMake in Release and MinSizeRel
// https://gcc.gnu.org/onlinedocs/gcc-4.6.1/gcc/Variadic-Macros.html
#ifndef NDEBUG
#define DEBUG(fmt, ...)                                                                            \
    fprintf(stderr, "[DEBUG]: %s:%d:%s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define WARNING(fmt, ...)                                                                          \
    fprintf(stderr, "[WARNING]: %s:%d:%s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#define ERROR(fmt, ...)                                                                            \
    fprintf(stderr, "[ERROR]: %s:%d:%s(): " fmt "\n", __FILE__, __LINE__, __func__, ##__VA_ARGS__)
#else
#define DEBUG(fmt, ...)
#define WARNING(fmt, ...)
#define ERROR(fmt, ...)
#endif

#endif /* WBT_DEBUG_H */
