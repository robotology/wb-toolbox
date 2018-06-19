# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 2.6)

set(PROJECTNAME sharedlib++)
project(${PROJECTNAME})


# Create the library 
file(GLOB folder_source ./src/*.cpp)
file(GLOB_RECURSE folder_header ./include/*.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})

include_directories(./include)
add_library(${PROJECTNAME} ${folder_source} ${folder_header} )
target_link_libraries(${PROJECTNAME})

# Create examples
option (BUILD_EXAMPLES "Build examples" ON)
if (BUILD_EXAMPLES)
    add_subdirectory(examples)
endif(BUILD_EXAMPLES)

