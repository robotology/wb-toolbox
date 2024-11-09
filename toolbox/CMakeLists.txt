# Copyright (C) 2018 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(sharedlibpp REQUIRED)
find_package(BlockFactory COMPONENTS Core REQUIRED)

add_subdirectory(base)
add_subdirectory(library)

install_basic_package_files(WBToolbox
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY AnyNewerVersion
    EXPORT WBToolboxExport
    DEPENDENCIES BlockFactoryCore
    NAMESPACE WBToolbox::
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
