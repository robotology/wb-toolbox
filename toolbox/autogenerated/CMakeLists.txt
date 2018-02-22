cmake_policy(SET CMP0048 NEW)
project(ClockServer LANGUAGES CXX VERSION 0.1)
cmake_minimum_required(VERSION 3.0.2)

# Configure the project
# =====================
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(GNUInstallDirs)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Use a general prefix for the project
set(VARS_PREFIX ${PROJECT_NAME})

# Build the library
# =================

# Export the includes needed to who inherits this library
# Set this up properly for handling either BUILD and INSTALL trees
set(${VARS_PREFIX}_INCLUDE_BUILD ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(${VARS_PREFIX}_INCLUDE_INSTALL ${CMAKE_INSTALL_INCLUDEDIR}/${VARS_PREFIX})

# Add the target
add_library(${VARS_PREFIX} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/include/thrift/ClockServer.h
                                  ${CMAKE_CURRENT_SOURCE_DIR}/src/thrift/ClockServer.cpp)

if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set_target_properties(${VARS_PREFIX} PROPERTIES COMPILE_FLAGS "-fPIC")
endif()

# Setup the include directories
target_include_directories(${VARS_PREFIX} PUBLIC
                           $<BUILD_INTERFACE:${${VARS_PREFIX}_INCLUDE_BUILD}>
                           $<INSTALL_INTERFACE:${${VARS_PREFIX}_INCLUDE_INSTALL}>)

# YARP
find_package(YARP REQUIRED)
target_include_directories(${VARS_PREFIX} PUBLIC ${YARP_INCLUDE_DIRS})
