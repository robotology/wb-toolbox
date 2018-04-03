#cmake_policy(SET CMP0048 NEW)
#project(ClockServer LANGUAGES CXX VERSION 0.1)
#cmake_minimum_required(VERSION 3.0.2)

# TODO: sooner or later ClockServer's thrift will be installed by YARP. In this case
#       it will be required only generating the library while compiling.

# CONFIGURE THE PROJECT
# =====================
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(GNUInstallDirs)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# BUILD THE LIBRARY
# =================

# Add the target
add_library(ClockServer STATIC ${CMAKE_CURRENT_SOURCE_DIR}/include/thrift/ClockServer.h
                               ${CMAKE_CURRENT_SOURCE_DIR}/src/thrift/ClockServer.cpp)

# Needed for static libraries
set_target_properties(ClockServer PROPERTIES POSITION_INDEPENDENT_CODE ON)

# YARP
find_package(YARP REQUIRED)

# Setup the include directories
target_include_directories(ClockServer PUBLIC
                           ${YARP_INCLUDE_DIRS}
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(TARGETS ClockServer
        EXPORT WBToolboxSimulinkCoder
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/ClockServer
)

set_property(GLOBAL APPEND PROPERTY WBToolboxSimulinkCoderExport_TARGETS ClockServer)
