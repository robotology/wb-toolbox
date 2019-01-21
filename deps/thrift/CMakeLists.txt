# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# GENERATE THE LIBRARY
# ====================

find_package(YARP 3.0.0 REQUIRED)

# TODO: sooner or later this file will be stored in YARP
yarp_add_idl(CLOCK_RPC_FILES clock_rpc.thrift)

# BUILD THE LIBRARY
# =================

# Add the library target
add_library(ClockRpc ${CLOCK_RPC_FILES} clock_rpc.thrift)

# Needed for static libraries
set_target_properties(ClockRpc PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Link it with YARP
find_package(YARP 3.0.0 REQUIRED)
target_link_libraries(ClockRpc YARP::YARP_OS)

# Extract the include directory from the files names
foreach(file ${CLOCK_RPC_FILES})
    STRING(REGEX MATCH ".+\\.h?h$" file ${file})
    if(file)
        get_filename_component(include_dir ${file} DIRECTORY)
        list(APPEND CLOCKRPC_INCLUDE_DIRS ${include_dir})
    endif()
endforeach()

# Setup the include directories
target_include_directories(ClockRpc PUBLIC
    $<BUILD_INTERFACE:${CLOCKRPC_INCLUDE_DIRS}>)

install(
    TARGETS ClockRpc
    EXPORT WBToolboxExport
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
