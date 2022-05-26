/*
 * strprintf.h
 *
 *  Created on: 21.05.2020
 *      Author: eyck
 */

#ifndef SCC_INCL_UTIL_STRPRINTF_H_
#define SCC_INCL_UTIL_STRPRINTF_H_

#include <cstdarg>
#include <iostream>
#include <string>
#include <vector>
#ifdef MSVC
#define _CRT_NO_VA_START_VALIDATION
#endif

/**
 * \ingroup scc-common
 */
/**@{*/
//! @brief SCC common utilities
namespace util {
//! allocate and print to a string buffer
inline std::string strprintf(const std::string format, ...) {
    va_list args;
    va_start(args, format);
    size_t len = std::vsnprintf(NULL, 0, format.c_str(), args);
    va_end(args);
    std::vector<char> vec(len + 1);
    va_start(args, format);
    std::vsnprintf(&vec[0], len + 1, format.c_str(), args);
    va_end(args);
    return vec.data();
}
} // namespace util
/**@}*/
#endif /* SCC_INCL_UTIL_STRPRINTF_H_ */
