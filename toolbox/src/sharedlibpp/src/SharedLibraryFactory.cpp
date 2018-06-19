// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2013 iCub Facility
 * Authors: Paul Fitzpatrick
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */

#include <SharedLibraryFactory.h>

shlibpp::SharedLibraryFactory::SharedLibraryFactory() :
        status(STATUS_NONE),
        returnValue(0),
        rct(1) {
}

shlibpp::SharedLibraryFactory::SharedLibraryFactory(const char *dll_name,
                                                     const char *fn_name) :
        status(STATUS_NONE),
        returnValue(0),
        rct(1) {
    open(dll_name,fn_name);
}

shlibpp::SharedLibraryFactory::~SharedLibraryFactory() {
}

bool shlibpp::SharedLibraryFactory::open(const char *dll_name, const char *fn_name) {
    returnValue = 0;
    name = "";
    status = STATUS_NONE;
    api.startCheck = 0;
    if (!lib.open(dll_name)) {        
        status = STATUS_LIBRARY_NOT_LOADED;        
        return false;
    }
    void *fn = lib.getSymbol((fn_name!=0/*NULL*/)?fn_name:SHLIBPP_DEFAULT_FACTORY_NAME);
    if (fn==0/*NULL*/) {
        lib.close();
        status = STATUS_FACTORY_NOT_FOUND;
        return false;
    }
    useFactoryFunction(fn);
    if (!isValid()) {
        status = STATUS_FACTORY_NOT_FUNCTIONAL;
        return false;
    }
    status = STATUS_OK;
    name = dll_name;
    return true;
}

bool shlibpp::SharedLibraryFactory::isValid() const {
    if (returnValue != VOCAB4('S','H','P','P')) {
        return false;
    }
    if (api.startCheck != VOCAB4('S','H','P','P')) {
        return false;
    }
    if (api.structureSize != sizeof(SharedLibraryClassApi)) {
        return false;
    }
    if (api.systemVersion != 3) {
        return false;
    }
    if (api.endCheck != VOCAB4('P','L','U','G')) {
        return false;
    }
    return true;
}

int shlibpp::SharedLibraryFactory::getStatus() const {
    return status;
}
const shlibpp::SharedLibraryClassApi& shlibpp::SharedLibraryFactory::getApi() const {
    return api;
}

int shlibpp::SharedLibraryFactory::getReferenceCount() const {
    return rct;
}


int shlibpp::SharedLibraryFactory::addRef() {
    rct++;
    return rct;
}

int shlibpp::SharedLibraryFactory::removeRef() {
    rct--;
    return rct;
}

std::string shlibpp::SharedLibraryFactory::getName() const {
    return name;
}


std::string shlibpp::SharedLibraryFactory::getLastNativeError() const {
    return lib.getLastNativeError();
}


bool shlibpp::SharedLibraryFactory::useFactoryFunction(void *factory) {
    api.startCheck = 0;
    if (factory == NULL) {
        return false;
    }
    isValid();
    returnValue =
        ((int (*)(void *ptr,int len)) factory)(&api,sizeof(SharedLibraryClassApi));
    return isValid();
}
