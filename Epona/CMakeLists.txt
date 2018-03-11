# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

include(EponaConfig)

project(${EPONA_NAME})
set(CMAKE_CXX_STANDARD 14)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

if (NOT GLM_FOUND)
    add_subdirectory(${GLM_ROOT_DIR})
endif()

#Target
include_directories(
    ${EPONA_INCLUDE_DIR}
)

message(STATUS "Version: ${EPONA_VERSION}")

set(EPONA_HEADERS
    include/Epona/Debug.hpp
    include/Epona/Analysis.hpp
    include/Epona/HalfEdgeDataStructure.hpp
    include/Epona/FloatingPoint.hpp
    include/Epona/HyperPlane.hpp
    include/Epona/JacobiEigenvalue.hpp
    include/Epona/QuickhullConvexHull.hpp
)
set(EPONA_SOURCES
    sources/Analysis.cpp
    sources/HalfEdgeDataStructure.cpp
    sources/HyperPlane.cpp
    sources/JacobiEigenvalue.cpp
)

if (MSVC)
    configure_file(
        ${CMAKE_CURRENT_SOURCE_DIR}/EponaVersionInfo.rc.in
        ${CMAKE_CURRENT_BINARY_DIR}/EponaVersionInfo.rc

        @ONLY
    )

    set(EPONA_EXTRA
        ${EPONA_EXTRA}
        ${CMAKE_CURRENT_BINARY_DIR}/EponaVersionInfo.rc
    )
endif()

add_library(${EPONA_LIB}
    ${EPONA_HEADERS}
    ${EPONA_SOURCES}
)

if (EPONA_BUILD_DEBUG)
    target_compile_definitions(${EPONA_LIB} PUBLIC EPONA_DEBUG)
endif()

set_target_properties(
    ${EPONA_LIB}
    PROPERTIES
        VERSION ${EPONA_VERSION}
        SOVERSION ${EPONA_SOVERSION}
)

target_link_libraries(${EPONA_LIB})

install( DIRECTORY ${GLM_INCLUDE_DIR}/glm
    DESTINATION ${EPONA_INSTALL_INCLUDE_DIR}
    COMPONENT epona_extra
    FILES_MATCHING
        PATTERN "*.hpp"
            PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
        PATTERN "*.h"
            PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
        PATTERN "*.inl"
            PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)

install( TARGETS ${EPONA_LIB}
    COMPONENT epona_libraries
    RUNTIME DESTINATION ${EPONA_INSTALL_LIBRARY_DIR}
    LIBRARY DESTINATION ${EPONA_INSTALL_LIBRARY_DIR}
    ARCHIVE DESTINATION ${EPONA_INSTALL_LIBRARY_DIR}
)

install( DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
    DESTINATION ${EPONA_INSTALL_INCLUDE_DIR}
    COMPONENT epona_headers
    FILES_MATCHING PATTERN "*.hpp"
    PERMISSIONS OWNER_WRITE OWNER_READ GROUP_READ WORLD_READ
)

include(CMakePackageConfigHelpers)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/EponaMathConfigVersion.cmake
    VERSION ${EPONA_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/EponaMathConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/EponaMathConfig.cmake

    @ONLY
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/EponaMathConfig.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/EponaMathConfigVersion.cmake
    DESTINATION ${EPONA_INSTALL_LIBRARY_DIR}
    COMPONENT epona_headers
)
