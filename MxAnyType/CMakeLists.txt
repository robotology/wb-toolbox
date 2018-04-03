cmake_policy(SET CMP0048 NEW)
project(MxAnyType LANGUAGES CXX VERSION 0.1)
# set (PROJECT_VERSION 0.1) # TODO
cmake_minimum_required(VERSION 3.5)

# Configure the project
# =====================
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(GNUInstallDirs)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Use a general prefix for the project
# set(VARS_PREFIX ${PROJECT_NAME})
set(VARS_PREFIX "MxAnyType")

# TODO: when exporting this folder to a new repository, this will become
#       a new project. Uncomment the project line and the alternative
#       VARS_PREFIX.

# Build the library
# =================

# Add the target
add_library(${VARS_PREFIX} SHARED ${CMAKE_CURRENT_SOURCE_DIR}/include/AnyType.h
                                  ${CMAKE_CURRENT_SOURCE_DIR}/include/MxAnyType.h
                                  ${CMAKE_CURRENT_SOURCE_DIR}/src/MxAnyType.cpp)

set_target_properties(${VARS_PREFIX} PROPERTIES
        VERSION ${PROJECT_VERSION} # This is for the symlink of the .so
        PUBLIC_HEADER "include/AnyType.h;include/MxAnyType.h"
)

# Find Matlab resources
find_package(Matlab REQUIRED MX_LIBRARY)
target_include_directories(${VARS_PREFIX} SYSTEM PUBLIC ${Matlab_INCLUDE_DIRS})

# Setup the include directories
target_include_directories(${VARS_PREFIX} PUBLIC
                           $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                           $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${VARS_PREFIX}>)

# Build tests
# ===========

# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)

# Install
# =======

install(TARGETS MxAnyType
        #EXPORT WBToolboxSimulinkCoder
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/${VARS_PREFIX}
)

# TODO: when splitting this library from the toolbox, this will provide find_package support
# include(InstallBasicPackageFiles)
# install_basic_package_files(WBToolboxSimulinkCoder
#     VERSION ${PROJECT_VERSION}
#     COMPATIBILITY AnyNewerVersion
#     TARGETS_PROPERTY MxAnyType
#     NO_CHECK_REQUIRED_COMPONENTS_MACRO
# )
