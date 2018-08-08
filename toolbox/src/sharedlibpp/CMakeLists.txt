# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
project(sharedlibpp LANGUAGES CXX VERSION 2015.02.07)

include(GNUInstallDirs)

# C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Create the library 
file(GLOB folder_source src/*.cpp)
file(GLOB folder_header include/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

add_library(sharedlibpp ${folder_source} ${folder_header})

if(NOT WIN32)
    target_link_libraries(sharedlibpp dl)
endif()

target_include_directories(sharedlibpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/sharedlibpp>)

set_target_properties(sharedlibpp PROPERTIES
    VERSION ${PROJECT_VERSION}
    PUBLIC_HEADER ${folder_header})

install(
    TARGETS sharedlibpp
    EXPORT sharedlibpp
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/sharedlibpp")

include(InstallBasicPackageFiles)
install_basic_package_files(sharedlibpp
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT sharedlibpp
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
