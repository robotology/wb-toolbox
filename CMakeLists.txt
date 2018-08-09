# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(WB-Toolbox LANGUAGES CXX VERSION 4)

if(WBT_BUILD_DOCS)
    add_subdirectory(doc)
    return()
endif()

# C++ standard
set(CMAKE_CXX_STANDARD 11)
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
if(NOT BUILD_SHARED_LIBS)
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# Tweak linker flags in Linux
if(UNIX AND NOT APPLE)
    get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
    if(${LINKER_BIN} STREQUAL "ld")
        set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--unresolved-symbols=report-all") # TODO
    endif()
endif()

# Add custom functions / macros
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YCM 0.9 REQUIRED)
include(YCMDefaultDirs)

find_package(Matlab COMPONENTS
    MX_LIBRARY
    ENG_LIBRARY
    MAIN_PROGRAM
    # SIMULINK # Requires CMake > 3.7
)

if(NOT WBT_USES_MATLAB)
    option(WBT_USES_MATLAB "Compile also Matlab-related components" ${Matlab_FOUND})
endif()

if(WBT_USES_MATLAB)
    add_subdirectory(MxAnyType)
endif()

add_subdirectory(toolbox)

include(AddUninstallTarget)
