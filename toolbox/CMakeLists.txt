# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(shlibpp REQUIRED)
find_package(BlockFactory COMPONENTS BlockFactoryCore REQUIRED)

add_subdirectory(base)
add_subdirectory(library)

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
