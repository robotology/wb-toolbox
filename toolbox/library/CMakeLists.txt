# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# ==============
# WBTOOLBOX BASE
# ==============

# Add configure_block macro
include(Utilities)

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

# =================
# WBTOOLBOX LIBRARY
# =================

set(WBTOOLBOXLIBRARY_EXT_LIBRARIES "")

# configure_block(BLOCK_NAME "Inverse Kinematics"
#     GROUP "Model"
#     LIST_PREFIX WBT
#     SOURCES src/RemoteInverseKinematics.cpp
#     HEADERS include/RemoteInverseKinematics.h)

option(WBT_USES_ICUB "Build models which need iCub library (e.g. Minimum Jerk Traj. Generator)" ON)

if(WBT_USES_ICUB)
    find_package(ICUB REQUIRED)

    configure_block(BLOCK_NAME "Minimum Jerk Trajectory Generator"
        GROUP "Utilities"
        LIST_PREFIX WBT
        SOURCES src/MinimumJerkTrajectoryGenerator.cpp
        HEADERS include/Block/MinimumJerkTrajectoryGenerator.h)

    configure_block(BLOCK_NAME "Discrete Filter"
        GROUP "Utilities"
        LIST_PREFIX WBT
        SOURCES src/DiscreteFilter.cpp
        HEADERS include/Block/DiscreteFilter.h)

    # if (${ICUB_USE_IPOPT})
    #     find_package(iDynTree REQUIRED)
    #     add_definitions(-DWBT_USES_IPOPT)
    #     configure_block(BLOCK_NAME "Inverse Kinematics"
    #         GROUP "Model"
    #         LIST_PREFIX WBT
    #         SOURCES src/InverseKinematics.cpp
    #         HEADERS include/InverseKinematics.h)
    #
    #     include_directories(SYSTEM ${iKin_INCLUDE_DIRS})
    # endif()
endif()

# Search qpOASES and enable block which depends on it
find_package(qpOASES)
if(NOT WBT_USES_QPOASES)
    option(WBT_USES_QPOASES "Compile also qpOASES-related components" ${qpOASES_FOUND})
endif()

configure_block(BLOCK_NAME "Yarp Read"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/YarpRead.cpp
    HEADERS include/Block/YarpRead.h)

configure_block(BLOCK_NAME "Yarp Write"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/YarpWrite.cpp
    HEADERS include/Block/YarpWrite.h)

configure_block(BLOCK_NAME "Real Time Synchronizer"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/RealTimeSynchronizer.cpp
    HEADERS include/Block/RealTimeSynchronizer.h)

configure_block(BLOCK_NAME "Yarp Clock"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/YarpClock.cpp
    HEADERS include/Block/YarpClock.h)

if(WBT_USES_QPOASES)
    configure_block(BLOCK_NAME "qpOASES"
        GROUP "Utilities"
        LIST_PREFIX WBT
        SOURCES src/QpOases.cpp
        HEADERS include/Block/QpOases.h)
endif()

configure_block(BLOCK_NAME "Simulator Synchronizer"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/SimulatorSynchronizer.cpp
    HEADERS include/Block/SimulatorSynchronizer.h)
add_library(WBToolbox::ClockRPC ALIAS ClockRpc)
list(APPEND WBTOOLBOXLIBRARY_EXT_LIBRARIES "ClockRpc")

configure_block(BLOCK_NAME "Mass Matrix"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/MassMatrix.cpp
    HEADERS include/Block/MassMatrix.h)

configure_block(BLOCK_NAME "Inverse Dynamics"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/InverseDynamics.cpp
    HEADERS include/Block/InverseDynamics.h)

configure_block(BLOCK_NAME "Centroidal Momentum"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/CentroidalMomentum.cpp
    HEADERS include/Block/CentroidalMomentum.h)

configure_block(BLOCK_NAME "Forward Kinematics"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/ForwardKinematics.cpp
    HEADERS include/Block/ForwardKinematics.h)

configure_block(BLOCK_NAME "Jacobian"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/Jacobian.cpp
    HEADERS include/Block/Jacobian.h)

configure_block(BLOCK_NAME "DotJ Nu"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/DotJNu.cpp
    HEADERS include/Block/DotJNu.h)

configure_block(BLOCK_NAME "Relative Transform"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/RelativeTransform.cpp
    HEADERS include/Block/RelativeTransform.h)

configure_block(BLOCK_NAME "Set References"
    GROUP "Actuators"
    LIST_PREFIX WBT
    SOURCES src/SetReferences.cpp
    HEADERS include/Block/SetReferences.h)

configure_block(BLOCK_NAME "Set Motor Parameters"
    GROUP "Actuators"
    LIST_PREFIX WBT
    SOURCES src/SetMotorParameters.cpp
    HEADERS include/Block/SetMotorParameters.h)

configure_block(BLOCK_NAME "Get Measurement"
    GROUP "State"
    LIST_PREFIX WBT
    SOURCES src/GetMeasurement.cpp
    HEADERS include/Block/GetMeasurement.h)

configure_block(BLOCK_NAME "Get Limits"
    GROUP "State"
    LIST_PREFIX WBT
    SOURCES src/GetLimits.cpp
    HEADERS include/Block/GetLimits.h)

get_property(ALL_LIBRARY_HEADERS GLOBAL PROPERTY WBT_HEADERS)
get_property(ALL_LIBRARY_SOURCES GLOBAL PROPERTY WBT_SOURCES)

add_library(WBToolboxLibrary SHARED
    src/Factory.cpp
    ${ALL_LIBRARY_HEADERS}
    ${ALL_LIBRARY_SOURCES})
add_library(WBToolbox::Library ALIAS WBToolboxLibrary)

if(MSVC)
    # Import math symbols from standard cmath
    target_compile_definitions(WBToolboxLibrary PRIVATE "_USE_MATH_DEFINES")
endif()

target_include_directories(WBToolboxLibrary PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/Block>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/WBToolbox>)

target_include_directories(WBToolboxLibrary SYSTEM PRIVATE ${EIGEN3_INCLUDE_DIR})

set_target_properties(WBToolboxLibrary PROPERTIES
    PUBLIC_HEADER "${ALL_LIBRARY_HEADERS}"
    OUTPUT_NAME "WBToolboxLibrary")

list(APPEND WBTOOLBOXLIBRARY_EXT_LIBRARIES
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_math
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-modelio-urdf
    iDynTree::idyntree-high-level)

target_link_libraries(WBToolboxLibrary PUBLIC WBToolboxBase)
target_link_libraries(WBToolboxLibrary PRIVATE BlockFactory::Core shlibpp::shlibpp ${WBTOOLBOXLIBRARY_EXT_LIBRARIES})

if(WBT_USES_ICUB)
    target_compile_definitions(WBToolboxLibrary PRIVATE "WBT_USES_ICUB")
    target_link_libraries(WBToolboxLibrary PUBLIC ctrlLib)

    # Manually include iCub include folders. They are not exported by the linked library.
    target_include_directories(WBToolboxLibrary PRIVATE ${ctrlLib_INCLUDE_DIRS})

#    if (${ICUB_USE_IPOPT})
#        target_link_libraries(WBToolboxLibrary PUBLIC iKin)
#        include_directories(${iKin_INCLUDE_DIRS})
#    endif()
endif()

if(WBT_USES_QPOASES)
    target_compile_definitions(WBToolboxLibrary PUBLIC "WBT_USES_QPOASES")
    target_link_libraries(WBToolboxLibrary PUBLIC ${qpOASES_LIBRARIES})

    # Manually include qpOASES include folders. They are not exported by the linked library.
    target_include_directories(WBToolboxLibrary PRIVATE ${qpOASES_INCLUDE_DIRS})
endif()

install(
    TARGETS WBToolboxBase WBToolboxLibrary
    EXPORT WBToolboxExport
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/WBToolbox/Block)

if(WBT_USES_ICUB)
    list(APPEND WBTOOLBOX_LIBRARY_PUBLIC_DEPENDS iCub)
endif()
if(WBT_USES_QPOASES)
    list(APPEND WBTOOLBOX_LIBRARY_PUBLIC_DEPENDS qpOASES)
endif()

install_basic_package_files(WBToolbox
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT WBToolboxExport
    FIRST_TARGET WBToolboxBase
    DEPENDENCIES YARP iDynTree ${WBTOOLBOX_LIBRARY_PUBLIC_DEPENDS} BlockFactoryCore
    NAMESPACE WBToolbox::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
