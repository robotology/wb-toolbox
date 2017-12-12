# Copyright (C) 2013-2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Jorhabib Eljaik, Francesco Romano, Diego Ferigo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 3.3)
project(WB-Toolbox)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(WB-TOOLBOX_SHARE_DIR "${CMAKE_INSTALL_PREFIX}/share/WB-Toolbox")

add_subdirectory(MxAnyType)
add_subdirectory(toolbox)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/images DESTINATION ${WB-TOOLBOX_SHARE_DIR})

include(AddUninstallTarget)
