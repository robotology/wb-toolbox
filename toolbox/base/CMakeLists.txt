# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(WBTBASE_PUBLIC_HDR
    include/WBToolbox/Base/RobotInterface.h
    include/WBToolbox/Base/Configuration.h
    include/WBToolbox/Base/WholeBodySingleton.h
    include/WBToolbox/Base/WBBlock.h)

add_library(WBToolboxBase
    src/RobotInterface.cpp
    src/Configuration.cpp
    src/WholeBodySingleton.cpp
    src/WBBlock.cpp
    ${WBTBASE_PUBLIC_HDR})

set_target_properties(WBToolboxBase PROPERTIES
    PUBLIC_HEADER "${WBTBASE_PUBLIC_HDR}")

target_link_libraries(WBToolboxBase
    BlockFactory::Core
    YARP::YARP_os
    YARP::YARP_init
    YARP::YARP_dev
    iDynTree::idyntree-model
    iDynTree::idyntree-modelio
    iDynTree::idyntree-high-level
    Eigen3::Eigen)

target_include_directories(WBToolboxBase PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

install(
    TARGETS WBToolboxBase
    EXPORT WBToolboxExport
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/WBToolbox/Base)
