# Copyright (C) 2013-2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Jorhabib Eljaik, Francesco Romano, Diego Ferigo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

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
