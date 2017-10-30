cmake_policy(SET CMP0048 NEW)
project(MxAnyType LANGUAGES CXX VERSION 0.1)
cmake_minimum_required(VERSION 3.0.2)

# Configure the project
# =====================
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
include(GNUInstallDirs)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# Use a general prefix for the project
set(VARS_PREFIX ${PROJECT_NAME})
# set(VARS_PREFIX "MxAnyType")

# Build the library
# =================

# Export the includes needed to who inherits this library
# Set this up properly for handling either BUILD and INSTALL trees
set(${VARS_PREFIX}_INCLUDE_BUILD ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(${VARS_PREFIX}_INCLUDE_INSTALL ${CMAKE_INSTALL_INCLUDEDIR}/${VARS_PREFIX})

# Add the target
add_library(${VARS_PREFIX} STATIC ${CMAKE_CURRENT_SOURCE_DIR}/include/AnyType.h
                                  ${CMAKE_CURRENT_SOURCE_DIR}/include/MxAnyType.h
                                  ${CMAKE_CURRENT_SOURCE_DIR}/src/MxAnyType.cpp)


# TODO: temporary, waiting the library becomes a shared
if (CMAKE_SYSTEM_PROCESSOR STREQUAL "x86_64")
    set_target_properties(${VARS_PREFIX} PROPERTIES COMPILE_FLAGS "-fPIC")
endif()

# Find Eigen
find_package(Eigen3 REQUIRED)
target_include_directories(${VARS_PREFIX} PUBLIC "${EIGEN3_INCLUDE_DIR}")

# Find Matlab resources
find_package(Matlab REQUIRED MX_LIBRARY)
# target_include_directories(${VARS_PREFIX} SYSTEM PUBLIC "${Matlab_INCLUDE_DIRS}") // TODO: when committing
target_include_directories(${VARS_PREFIX} PUBLIC "${Matlab_INCLUDE_DIRS}")

# Setup the include directories
# TODO why in the how to was INTERFACE?
target_include_directories(${VARS_PREFIX} PUBLIC
                           $<BUILD_INTERFACE:${${VARS_PREFIX}_INCLUDE_BUILD}>
                           $<INSTALL_INTERFACE:${${VARS_PREFIX}_INCLUDE_INSTALL}>)

# Assign some useful properties
# set_target_properties(${VARS_PREFIX} PROPERTIES VERSION ${PROJECT_VERSION}
#                                      PUBLIC_HEADER MxAnyType.h)

# Build tests
# ===========
# add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/tests)
