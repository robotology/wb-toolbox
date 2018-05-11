# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(WB-Toolbox LANGUAGES CXX VERSION 3.1)

if(WBTOOLBOX_BUILD_DOCS)
    add_subdirectory(doc)
    return()
endif()

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Tweak linker flags in Linux
if(UNIX AND NOT APPLE)
    get_filename_component(LINKER_BIN ${CMAKE_LINKER} NAME)
    if(${LINKER_BIN} STREQUAL "ld")
        set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--unresolved-symbols=report-all")
    endif()
endif()

# Add custom functions / macros
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

find_package(Matlab QUIET COMPONENTS
    MX_LIBRARY
    ENG_LIBRARY
    MAIN_PROGRAM
    # SIMULINK # Requires CMake > 3.7
)

if(NOT WBTOOLBOX_USES_MATLAB)
    option(WBTOOLBOX_USES_MATLAB "Compile also Matlab-related components" ${Matlab_FOUND})
endif()

if(${Matlab_FOUND})
    add_subdirectory(MxAnyType)
endif()

add_subdirectory(toolbox)

include(AddUninstallTarget)
