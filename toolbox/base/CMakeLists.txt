# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_library(WBToolboxBase
    src/RobotInterface.cpp
    src/Configuration.cpp
    src/WholeBodySingleton.cpp
    src/WBBlock.cpp
    include/Base/RobotInterface.h
    include/Base/Configuration.h
    include/Base/WholeBodySingleton.h
    include/Base/WBBlock.h)

target_link_libraries(WBToolboxBase
    BlockFactory::Core
    YARP::YARP_OS
    YARP::YARP_init
    YARP::YARP_dev
    iDynTree::idyntree-model
    iDynTree::idyntree-modelio-urdf
    iDynTree::idyntree-high-level)

target_include_directories(WBToolboxBase PUBLIC
    $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/WBToolbox/Base>)

install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/Base
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/WBToolbox)

install(
    TARGETS WBToolboxBase
    EXPORT WBToolboxExport
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/WBToolbox/Block)
