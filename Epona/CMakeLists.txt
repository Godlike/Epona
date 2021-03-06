# Copyright (C) 2017-2018 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

include(EponaConfig)

project(${EPONA_NAME})

set(EPONA_LIB_ALIAS Epona::Math)

add_library(${PROJECT_NAME} "")
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})
add_library(${EPONA_LIB_ALIAS} ALIAS ${PROJECT_NAME})

set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 17)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build (Debug or Release)" FORCE)
endif()

if (NOT GLM_FOUND)
    add_subdirectory(${GLM_ROOT_DIR})
endif()

#Target
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

target_sources(${PROJECT_NAME}
    PRIVATE
        ${EPONA_HEADERS}
        ${EPONA_SOURCES}
)

if (EPONA_BUILD_DEBUG)
    target_compile_definitions(${PROJECT_NAME}
        PUBLIC
            EPONA_DEBUG
    )
endif()

target_include_directories(${PROJECT_NAME}
    PUBLIC
        ${EPONA_INCLUDE_DIR}
)

set_target_properties(
    ${PROJECT_NAME}
    PROPERTIES
        VERSION ${EPONA_VERSION}
        SOVERSION ${EPONA_SOVERSION}
)

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

install( TARGETS ${PROJECT_NAME}
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
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${EPONA_VERSION}
    COMPATIBILITY SameMajorVersion
)

configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake.gen

    @ONLY
)

file(GENERATE
    OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INPUT ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake.gen
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${EPONA_INSTALL_LIBRARY_DIR}
    COMPONENT epona_headers
)
