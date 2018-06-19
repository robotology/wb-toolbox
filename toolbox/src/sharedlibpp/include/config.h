// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*
 * Copyright (C) 2011 Ali Paikan
 * Authors: Ali Paikan
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 *
 */


#ifndef _SHLIBPP_CONFIG_
#define _SHLIBPP_CONFIG_

#if defined(WIN32)
#	define SHLIBPP_FILTER_API
#endif 

#if defined _WIN32 || defined __CYGWIN__
#  define SHLIBPP_HELPER_DLL_IMPORT __declspec(dllimport)
#  define SHLIBPP_HELPER_DLL_EXPORT __declspec(dllexport)
#  define SHLIBPP_HELPER_DLL_LOCAL
#  define SHLIBPP_HELPER_DLL_IMPORT_EXTERN extern
#  define SHLIBPP_HELPER_DLL_EXPORT_EXTERN
#  ifndef SHLIBPP_NO_DEPRECATED_WARNINGS
#    define SHLIBPP_HELPER_DLL_DEPRECATED __declspec(deprecated)
#  endif
#else
#  if __GNUC__ >= 4
#    define SHLIBPP_HELPER_DLL_IMPORT __attribute__ ((visibility("default")))
#    define SHLIBPP_HELPER_DLL_EXPORT __attribute__ ((visibility("default")))
#    define SHLIBPP_HELPER_DLL_LOCAL  __attribute__ ((visibility("hidden")))
#    define SHLIBPP_HELPER_DLL_IMPORT_EXTERN
#    define SHLIBPP_HELPER_DLL_EXPORT_EXTERN
#  else
#     define SHLIBPP_HELPER_DLL_IMPORT
#     define SHLIBPP_HELPER_DLL_EXPORT
#     define SHLIBPP_HELPER_DLL_LOCAL
#     define SHLIBPP_HELPER_DLL_IMPORT_EXTERN
#     define SHLIBPP_HELPER_DLL_EXPORT_EXTERN
#  endif
#  if (__GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 1))
#    ifndef SHLIBPP_NO_DEPRECATED_WARNINGS
#      define SHLIBPP_HELPER_DLL_DEPRECATED __attribute__ ((__deprecated__))
#    endif
#  endif
#endif

#ifndef SHLIBPP_HELPER_DLL_DEPRECATED
#  define SHLIBPP_HELPER_DLL_DEPRECATED
#endif

#if defined SHLIBPP_FILTER_API
#  define SHLIBPP_IMPORT SHLIBPP_HELPER_DLL_IMPORT
#  define SHLIBPP_EXPORT SHLIBPP_HELPER_DLL_EXPORT
#  define SHLIBPP_LOCAL SHLIBPP_HELPER_DLL_LOCAL
#  define SHLIBPP_IMPORT_EXTERN SHLIBPP_HELPER_DLL_IMPORT_EXTERN
#  define SHLIBPP_EXPORT_EXTERN SHLIBPP_HELPER_DLL_EXPORT_EXTERN
#  define SHLIBPP_DEPRECATED SHLIBPP_HELPER_DLL_DEPRECATED
#else
#  define SHLIBPP_IMPORT
#  define SHLIBPP_EXPORT
#  define SHLIBPP_LOCAL
#  define SHLIBPP_IMPORT_EXTERN
#  define SHLIBPP_EXPORT_EXTERN
#  define SHLIBPP_DEPRECATED
#endif

#define SHLIBPP_POINTER_SIZE    8
#define SHLIBPP_SHAREDLIBRARYCLASSAPI_PADDING (30-2*(SHLIBPP_POINTER_SIZE/4))

#endif //_SHLIBPP_CONFIG_
