/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef _SHLIBPP_SHAREDLIBRARY_
#define _SHLIBPP_SHAREDLIBRARY_

#include <string>

namespace shlibpp {
    class SharedLibrary;
}

/**
 * Low-level wrapper for loading shared libraries (DLLs) and accessing
 * symbols within it.
 */
class shlibpp::SharedLibrary
{
public:
    /**
     * Initialize, without opening a shared library yet.
     */
    SharedLibrary();

    /**
     * Load the named shared library / DLL.
     *
     * @param filename name of file (see open method)
     */
    SharedLibrary(const char* filename);

    /**
     * Destructor.  Will close() if needed.
     */
    virtual ~SharedLibrary();

    /**
     * Load the named shared library / DLL.  The library is found
     * using the algoithm of ACE::ldfind.  Operating-system-specific
     * extensions will be tried, and the relevant path for shared
     * libraries.
     *
     * @param filename name of file.
     * @return true on success
     */
    bool open(const char* filename);

    /**
     * Shared library no longer needed, unload if not in use elsewhere.
     * @return true on success
     */
    bool close();

    /**
     * Look up a symbol in the shared library.
     */
    void* getSymbol(const char* symbolName);

    /**
     * Check if the shared library is valid
     *
     * @return true iff a valid library has been loaded.
     */
    bool isValid() const;

    /**
     * Get Last error message reported by the Os (if presented)
     * @return return error message
     */
    std::string getLastNativeError() const;

private:
    void* implementation;
    std::string err_message;
    SharedLibrary(const SharedLibrary&); // Not implemented
    SharedLibrary& operator=(const SharedLibrary&); // Not implemented
};

#endif
