# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(WB-Toolbox LANGUAGES CXX VERSION 5)

if(WBT_BUILD_DOCS)
    add_subdirectory(doc)
    return()
endif()

# =================
# FIND DEPENDENCIES
# =================

# Matlab is not a compile dependency. If it is found, the Simulink library
# and all the Matlab-related resources are installed.
find_package(Matlab COMPONENTS
    MX_LIBRARY
    ENG_LIBRARY
    MAIN_PROGRAM
    # SIMULINK # Requires CMake > 3.7
)

find_package(YCM 0.9 REQUIRED)
find_package(YARP 3.0.0 REQUIRED)

# Fail if YARP is not compiled as shared library
# see https://github.com/robotology/codyco-modules/issues/44
if(NOT ${YARP_IS_SHARED_LIBRARY})
    message(FATAL_ERROR "YARP was found, but it was compiled as static library. A shared library version of YARP is required.")
endif()

find_package(iDynTree 1.0.0 REQUIRED)

# This find loads Eigen.
# It would be nice using the EigenConfig shipped with eigen package, but
# on Ubuntu 16.04 eigen is old and provides only FindEigen.
find_package(Eigen3 REQUIRED)

# ====================
# OPTIONS AND INCLUDES
# ====================

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build type
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Release" CACHE STRING
            "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(WBT_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${WBT_BUILD_TYPES})
endif()

# Libraries type
if(MSVC)
    option(WBT_SHARED_LIBRARY "Compile WBToolbox as a shared library" FALSE)
else()
    option(WBT_SHARED_LIBRARY "Compile WBToolbox as a shared library" TRUE)
endif()
set(BUILD_SHARED_LIBS ${WBT_SHARED_LIBRARY})
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Tweak linker flags in Linux
if(UNIX AND NOT APPLE)
    if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
        get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
            if(${LINKER_BIN} STREQUAL "ld")
                set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--unresolved-symbols=report-all")
            endif()
    endif()
endif()

# Settings for RPATH
if(NOT MSVC)
    option(WBT_ENABLE_RPATH "Enable RPATH installation" TRUE)
    mark_as_advanced(WBT_ENABLE_RPATH)
endif()

# Export all symbols in Windows
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Add a postfix to Windows libraries compiled in debug
if(MSVC)
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

# Control where binaries and libraries are placed in the build folder.
# This simplifies tests running in Windows.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS ${CMAKE_INSTALL_PREFIX}/bin
                          ${CMAKE_INSTALL_PREFIX}/lib
                          DEPENDS WBT_ENABLE_RPATH
                          USE_LINK_PATH)

# Get include-what-you-see information when compiling
option(WBT_USE_IWYU "Get the output of include-what-you-use" OFF)
if(WBT_USE_IWYU)
    find_program(IWYU_PATH NAMES include-what-you-use iwyu)
    if(IWYU_PATH)
        set(CMAKE_CXX_INCLUDE_WHAT_YOU_USE ${IWYU_PATH})
    endif()
endif()

# Add custom functions / macros
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(GNUInstallDirs)
include(YCMDefaultDirs)
include(InstallBasicPackageFiles)
include(AddUninstallTarget)

if(NOT WBT_USES_MATLAB)
    option(WBT_USES_MATLAB "Install Matlab / Simulink resources" ${Matlab_FOUND})
endif()

# ==============
# SUBDIRECTORIES
# ==============

add_subdirectory(deps)
add_subdirectory(toolbox)

if(WBT_USES_MATLAB)
    add_subdirectory(matlab)
endif()
