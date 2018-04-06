# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(WB-Toolbox LANGUAGES CXX VERSION 3.1)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add custom functions / macros
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

add_subdirectory(MxAnyType)
add_subdirectory(toolbox)

include(AddUninstallTarget)
