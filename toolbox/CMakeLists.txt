# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(shlibpp REQUIRED)
find_package(BlockFactory COMPONENTS BlockFactoryCore REQUIRED)
# =================
# FIND DEPENDENCIES
# =================

find_package(YARP 3.0.0 REQUIRED)

# Fail if YARP is not compiled as shared library
# see https://github.com/robotology/codyco-modules/issues/44
if(NOT ${YARP_IS_SHARED_LIBRARY})
    message(FATAL_ERROR "YARP was found, but it was compiled as static library. A shared library version of YARP is required.")
endif()

find_package(iDynTree 0.7.2 REQUIRED)

# This find loads Eigen.
# It would be nice using the EigenConfig shipped with eigen package, but
# on Ubuntu 16.04 eigen is old and provides only FindEigen.
find_package(Eigen3 REQUIRED)

# ====================
# OPTIONS AND INCLUDES
# ====================

# Add configure_block macro
include(Utilities)

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
                          LIB_DIRS ${CMAKE_INSTALL_PREFIX}/mex ${CMAKE_INSTALL_PREFIX}/lib
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

include(GNUInstallDirs)
include(InstallBasicPackageFiles)

# ===============
# TOOLBOX TARGETS
# ===============

add_subdirectory(library)

# =====================
# MAIN WBTOOLBOX TARGET
# =====================

# Dummy target
add_library(WBToolbox INTERFACE)

set(WBTOOLBOX_DEPENDENCIES YCM ToolboxCore ToolboxCoder WBToolboxLibrary)
if (WBT_USES_MATLAB)
    list(APPEND WBTOOLBOX_DEPENDENCIES ToolboxMex)
endif()

install(
    TARGETS WBToolbox
    EXPORT WBToolbox)

install_basic_package_files(WBToolbox
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT WBToolbox
    DEPENDENCIES ${WBTOOLBOX_DEPENDENCIES}
    NAMESPACE WBToolbox::
    INCLUDE_FILE ${CMAKE_SOURCE_DIR}/cmake/ExtraPackageConfigVars.cmake.in)
