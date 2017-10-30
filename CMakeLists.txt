# Copyright (C) 2013-2015 Instituto Italiano di Tecnologia
# Author: Jorhabib Eljaik, Francesco Romano
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.11)

project(WB-Toolbox)

find_package(YCM REQUIRED)
include(YCMDefaultDirs)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
set(WB-TOOLBOX_SHARE_DIR "${CMAKE_INSTALL_PREFIX}/share/WB-Toolbox")

add_subdirectory(MxAnyType)
add_subdirectory(toolbox)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/images DESTINATION ${WB-TOOLBOX_SHARE_DIR})

include(AddUninstallTarget)
