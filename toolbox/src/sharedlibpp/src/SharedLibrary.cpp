/*
 * Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT)
 * All rights reserved.
 *
 * This software may be modified and distributed under the terms of the
 * GNU Lesser General Public License v2.1 or any later version.
 */

#include <cstddef>

#if defined(WIN32)
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <SharedLibrary.h>

using namespace shlibpp;

SharedLibrary::SharedLibrary()
    : implementation(NULL)
{}

SharedLibrary::SharedLibrary(const char* filename)
    : implementation(NULL)
{
    open(filename);
}

SharedLibrary::~SharedLibrary()
{
    close();
}

bool SharedLibrary::open(const char* filename)
{
    err_message.clear();
    close();
#if defined(WIN32)
    implementation = (void*) LoadLibrary(filename);
    LPTSTR msg = NULL;
    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER
                      | FORMAT_MESSAGE_IGNORE_INSERTS,
                  NULL,
                  GetLastError(),
                  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                  (LPTSTR) &msg,
                  0,
                  NULL);

    if (msg != NULL) {
        err_message = std::string(msg);
        // release memory allocated by FormatMessage()
        LocalFree(msg);
        msg = NULL;
    }
    return (implementation != NULL);
#else
    implementation = dlopen(filename, RTLD_LAZY);
    char* msg = dlerror();
    if (msg)
        err_message = msg;
    return implementation != NULL;
#endif
}

bool SharedLibrary::close()
{
    if (implementation != NULL) {
#if defined(WIN32)
        FreeLibrary((HINSTANCE) implementation);
#else
        dlclose(implementation);
#endif
        implementation = NULL;
    }
    return true;
}

void* SharedLibrary::getSymbol(const char* symbolName)
{
    err_message.clear();
    if (implementation == NULL)
        return NULL;
#if defined(WIN32)
    FARPROC proc = GetProcAddress((HINSTANCE) implementation, symbolName);
    LPTSTR msg = NULL;
    FormatMessage(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER
                      | FORMAT_MESSAGE_IGNORE_INSERTS,
                  NULL,
                  GetLastError(),
                  MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                  (LPTSTR) &msg,
                  0,
                  NULL);

    if (msg != NULL) {
        err_message = std::string(msg);
        // release memory allocated by FormatMessage()
        LocalFree(msg);
        msg = NULL;
    }
    return (void*) proc;
#else
    dlerror();
    void* func = dlsym(implementation, symbolName);
    char* msg = dlerror();
    if (msg)
        err_message = msg;
    return func;
#endif
}

bool SharedLibrary::isValid() const
{
    return implementation != NULL;
}

std::string SharedLibrary::getLastNativeError() const
{
    return err_message;
}
