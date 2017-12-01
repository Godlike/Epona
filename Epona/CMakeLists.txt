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

add_library(${EPONA_LIB}
    ${EPONA_HEADERS}
    ${EPONA_SOURCES}
)

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
