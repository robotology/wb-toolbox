# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# =====================
# OPTIONAL DEPENDENCIES
# =====================

find_package(ICUB)
if(NOT WBT_USES_ICUB)
    option(WBT_USES_ICUB "Build blocks that depend on iCub library" ${ICUB_FOUND})
endif()

find_package(qpOASES)
if(NOT WBT_USES_QPOASES)
    option(WBT_USES_QPOASES "Build blocks that depend on qpOASES library" ${qpOASES_FOUND})
endif()

# ===================
# BLOCKFACTORY PLUGIN
# ===================

# YARP blocks
list(APPEND WBT_BLOCKS "YarpRead")
list(APPEND WBT_BLOCKS "YarpWrite")
list(APPEND WBT_BLOCKS "YarpClock")
list(APPEND WBT_BLOCKS "GetLimits")
list(APPEND WBT_BLOCKS "SetReferences")
list(APPEND WBT_BLOCKS "GetMeasurement")
list(APPEND WBT_BLOCKS "SetMotorParameters")
list(APPEND WBT_BLOCKS "SimulatorSynchronizer")

# iDynTree blocks
list(APPEND WBT_BLOCKS "MassMatrix")
list(APPEND WBT_BLOCKS "InverseDynamics")
list(APPEND WBT_BLOCKS "DotJNu")
list(APPEND WBT_BLOCKS "Jacobian")
list(APPEND WBT_BLOCKS "ForwardKinematics")
list(APPEND WBT_BLOCKS "RelativeTransform")
list(APPEND WBT_BLOCKS "CentroidalMomentum")
list(APPEND WBT_BLOCKS "CMM")
# Other
list(APPEND WBT_BLOCKS "RealTimeSynchronizer")

if(WBT_USES_ICUB)
    list(APPEND WBT_BLOCKS "DiscreteFilter")
    list(APPEND WBT_BLOCKS "MinimumJerkTrajectoryGenerator")
endif()

if(WBT_USES_QPOASES)
    list(APPEND WBT_BLOCKS "QpOases")
endif()

# Include CMake helpers provided by BlockFactory
include(BlockFactoryPlugin)

# Register the blocks
foreach(block ${WBT_BLOCKS})
    register_blockfactory_block(
        BLOCK_NAME ${block}
        PLUGIN_NAME WBToolbox
        SOURCES src/${block}.cpp
        HEADERS include/WBToolbox/Block/${block}.h)
endforeach()

# Create the target of the plugin library
add_blockfactory_plugin(WBToolbox
    EXTRA_SOURCES src/Factory.cpp)
add_library(WBToolbox::Plugin ALIAS WBToolbox)

# Set public headers
get_property(WBT_BLOCKS_HEADERS GLOBAL PROPERTY WBToolbox_HEADERS)
set_target_properties(WBToolbox PROPERTIES
    PUBLIC_HEADER "${WBT_BLOCKS_HEADERS}")

if(MSVC)
    # Import math symbols from standard cmath
    target_compile_definitions(WBToolbox PRIVATE "_USE_MATH_DEFINES")
endif()

# Configure the include directories
target_include_directories(WBToolbox PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)

# Main dependencies
list(APPEND WBTOOLBOXLIBRARY_EXT_LIBRARIES
    YARP::YARP_OS
    YARP::YARP_dev
    YARP::YARP_math
    iDynTree::idyntree-core
    iDynTree::idyntree-model
    iDynTree::idyntree-modelio-urdf
    iDynTree::idyntree-high-level)

# Additional dependency of the SimulatorSynchronizer block
add_library(WBToolbox::ClockRPC ALIAS ClockRpc)
list(APPEND WBTOOLBOXLIBRARY_EXT_LIBRARIES "ClockRpc")

# Link the plugin with the WBToolboxBase library and
# all the external dependencies
target_link_libraries(WBToolbox
    PUBLIC WBToolboxBase
    PRIVATE ${WBTOOLBOXLIBRARY_EXT_LIBRARIES})

if(WBT_USES_ICUB)
    target_compile_definitions(WBToolbox PRIVATE "WBT_USES_ICUB")
    target_link_libraries(WBToolbox PRIVATE ctrlLib)

    # Manually include iCub include folders. They are not exported by the linked library.
    target_include_directories(WBToolbox PRIVATE ${ctrlLib_INCLUDE_DIRS})
endif()

if(WBT_USES_QPOASES)
    target_compile_definitions(WBToolbox PRIVATE "WBT_USES_QPOASES")
    target_link_libraries(WBToolbox PRIVATE ${qpOASES_LIBRARIES})

    # Manually include qpOASES include folders. They are not exported by the linked library.
    target_include_directories(WBToolbox PRIVATE ${qpOASES_INCLUDE_DIRS})
endif()

# Install the plugin
install_blockfactory_plugin(WBToolbox)
