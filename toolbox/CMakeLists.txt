# Copyright (C) 2013-2017 iCub Facility - Istituto Italiano di Tecnologia
# Author: Jorhabib Eljaik, Francesco Romano, Diego Ferigo
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

find_package(YARP REQUIRED)

# Fail if YARP is not compiled as shared library
# see https://github.com/robotology/codyco-modules/issues/44
if(NOT ${YARP_IS_SHARED_LIBRARY})
    message(FATAL_ERROR "YARP was found, but it was compiled as static library. A shared library version of YARP is required.")
endif()

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
include(YarpInstallationHelpers)
yarp_configure_external_installation(codyco)

find_package(Matlab REQUIRED MX_LIBRARY
                             ENG_LIBRARY
                             MAIN_PROGRAM)

if(NOT Matlab_FOUND)
    message(FATAL_ERROR "Matlab not found")
endif()

# Settings for rpath
if(NOT MSVC)
    # Add the option to enable RPATH
    option(WB-TOOLBOX_ENABLE_RPATH "Enable RPATH installation" TRUE)
    mark_as_advanced(WB-TOOLBOX_ENABLE_RPATH)
endif(NOT MSVC)

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS ${CMAKE_INSTALL_PREFIX}/bin
                          LIB_DIRS ${CMAKE_INSTALL_PREFIX}/mex ${CMAKE_INSTALL_PREFIX}/lib
                          DEPENDS WB-TOOLBOX_ENABLE_RPATH
                          USE_LINK_PATH)

find_package(Eigen3 REQUIRED)
find_package(iDynTree 0.7.2 REQUIRED)

include(Utilities)

configure_block(BLOCK_NAME "Base"
    LIST_PREFIX WBT
    SOURCES src/base/toolbox.cpp
            src/base/Block.cpp
            src/base/BlockInformation.cpp
            src/base/WBBlock.cpp
            src/base/Log.cpp
            src/base/ToolboxSingleton.cpp
            src/base/factory.cpp
            src/base/SimulinkBlockInformation.cpp
            src/base/Signal.cpp
            src/base/Configuration.cpp
            src/base/RobotInterface.cpp
    HEADERS include/base/toolbox.h
            include/base/Block.h
            include/base/BlockInformation.h
            include/base/WBBlock.h
            include/base/Log.h
            include/base/ToolboxSingleton.h
            include/base/SimulinkBlockInformation.h
            include/base/Signal.h
            include/base/Configuration.h
            include/base/RobotInterface.h
            )

# configure_block(BLOCK_NAME "Inverse Kinematics"
#     GROUP "Model"
#     LIST_PREFIX WBT
#     SOURCES src/RemoteInverseKinematics.cpp
#     HEADERS include/RemoteInverseKinematics.h)

option(WBT_USES_ICUB "Build models which need iCub library (e.g. Minimum Jerk Traj. Generator)" ON)
if (WBT_USES_ICUB)
    find_package(ICUB REQUIRED)
    add_definitions(-DWBT_USES_ICUB)

    configure_block(BLOCK_NAME "Minimum Jerk Trajectory Generator"
        GROUP "Utilities"
        LIST_PREFIX WBT
        SOURCES src/MinimumJerkTrajectoryGenerator.cpp
        HEADERS include/MinimumJerkTrajectoryGenerator.h)

    configure_block(BLOCK_NAME "Discrete Filter"
        GROUP "Utilities"
        LIST_PREFIX WBT
        SOURCES src/DiscreteFilter.cpp
        HEADERS include/DiscreteFilter.h)

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

    include_directories(SYSTEM ${ctrlLib_INCLUDE_DIRS})
endif()

configure_block(BLOCK_NAME "Yarp Read"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/YarpRead.cpp
    HEADERS include/YarpRead.h)

configure_block(BLOCK_NAME "Yarp Write"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/YarpWrite.cpp
    HEADERS include/YarpWrite.h)

configure_block(BLOCK_NAME "Real Time Synchronizer"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/RealTimeSynchronizer.cpp
    HEADERS include/RealTimeSynchronizer.h)

configure_block(BLOCK_NAME "Model Partitioner"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/ModelPartitioner.cpp
    HEADERS include/ModelPartitioner.h)

configure_block(BLOCK_NAME "Yarp Clock"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/YarpClock.cpp
    HEADERS include/YarpClock.h)

configure_block(BLOCK_NAME "Simulator Synchronizer"
    GROUP "Utilities"
    LIST_PREFIX WBT
    SOURCES src/SimulatorSynchronizer.cpp
    HEADERS include/SimulatorSynchronizer.h)

configure_block(BLOCK_NAME "Mass Matrix"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/MassMatrix.cpp
    HEADERS include/MassMatrix.h)

configure_block(BLOCK_NAME "Inverse Dynamics"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/InverseDynamics.cpp
    HEADERS include/InverseDynamics.h)

configure_block(BLOCK_NAME "Centroidal Momentum"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/CentroidalMomentum.cpp
    HEADERS include/CentroidalMomentum.h)

configure_block(BLOCK_NAME "Forward Kinematics"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/ForwardKinematics.cpp
    HEADERS include/ForwardKinematics.h)

configure_block(BLOCK_NAME "Jacobian"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/Jacobian.cpp
    HEADERS include/Jacobian.h)

configure_block(BLOCK_NAME "DotJ Nu"
    GROUP "Model"
    LIST_PREFIX WBT
    SOURCES src/DotJNu.cpp
    HEADERS include/DotJNu.h)

configure_block(BLOCK_NAME "Set References"
    GROUP "Actuators"
    LIST_PREFIX WBT
    SOURCES src/SetReferences.cpp
    HEADERS include/SetReferences.h)

configure_block(BLOCK_NAME "Set Low-Level PIDs"
    GROUP "Actuators"
    LIST_PREFIX WBT
    SOURCES src/SetLowLevelPID.cpp
    HEADERS include/SetLowLevelPID.h)
#
configure_block(BLOCK_NAME "Get Measurement"
    GROUP "State"
    LIST_PREFIX WBT
    SOURCES src/GetMeasurement.cpp
    HEADERS include/GetMeasurement.h)

configure_block(BLOCK_NAME "Get Limits"
    GROUP "State"
    LIST_PREFIX WBT
    SOURCES src/GetLimits.cpp
    HEADERS include/GetLimits.h)

get_property(ALL_HEADERS GLOBAL PROPERTY WBT_HEADERS)
get_property(ALL_SOURCES GLOBAL PROPERTY WBT_SOURCES)

add_definitions(-DMATLAB_MEX_FILE)
include_directories(include)
include_directories(include/base)

include_directories(SYSTEM ${Matlab_INCLUDE_DIRS} "${Matlab_ROOT_DIR}/simulink/include")
include_directories(SYSTEM ${YARP_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

list(APPEND LINKED_LIBRARIES ${YARP_LIBRARIES})
list(APPEND LINKED_LIBRARIES ${iDynTree_LIBRARIES})

if (WBT_USES_ICUB)
    list(APPEND LINKED_LIBRARIES ctrlLib)
    if (${ICUB_USE_IPOPT})
        list(APPEND LINKED_LIBRARIES iKin)
    endif()
endif()

# Adding files used for the generation of the dynamic library
matlab_add_mex(
    NAME WBToolbox
    SRC ${ALL_HEADERS} ${ALL_SOURCES}
    LINK_TO ${LINKED_LIBRARIES}
)

# Link with MxAnyType library
# TODO: this will become an external project sooner or later
target_link_libraries(WBToolbox MxAnyType)

# Link with ClockServer library
add_subdirectory(autogenerated/)
target_link_libraries(WBToolbox ClockServer)

install(TARGETS WBToolbox DESTINATION ${CMAKE_INSTALL_PREFIX}/mex)

# The following line is to properly configure the installation script of the toolbox
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/matlab/startup_wbitoolbox.m.in
               ${CMAKE_BINARY_DIR}/startup_wbitoolbox.m)

# Custom script to generate the library to be committed on the repository
# This target is excluded from the normal build and must be called explicitly by the
# developer who modifies the library
add_custom_target(export_libraries ${Matlab_MAIN_PROGRAM} -nosplash -nodesktop -r export_library WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/matlab)
set_target_properties(export_libraries PROPERTIES EXCLUDE_FROM_DEFAULT_BUILD 1)

# Install configuration files
install(FILES ${CMAKE_BINARY_DIR}/startup_wbitoolbox.m DESTINATION ${WB-TOOLBOX_SHARE_DIR})
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/matlab/slblocks.m   DESTINATION ${WB-TOOLBOX_SHARE_DIR})

# Install the package folder
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/matlab/+WBToolbox
        DESTINATION ${WB-TOOLBOX_SHARE_DIR})

#if MAJOR >= 2014 && MINOR >= b
# Note: We had issues with Matlab 2014b and .mdl models.
# But this issue seems to have been disappeared in 2015b. We have to check if we need to enable this if again
# TODO: check if the mdl support is still required
# if (${Matlab_VERSION_STRING} GREATER "8.3")
    # install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/WBToolboxLibrary.slx DESTINATION ${WB-TOOLBOX_SHARE_DIR})
# else()
    # install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/WBToolboxLibrary.mdl DESTINATION ${WB-TOOLBOX_SHARE_DIR})
# endif()
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/matlab/library/exported/WBToolboxLibrary.slx
        DESTINATION ${WB-TOOLBOX_SHARE_DIR})
