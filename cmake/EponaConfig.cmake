# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

set(EPONA_NAME "EponaMath" CACHE STRING "Epona project name.")

set(CMAKE_CXX_STANDARD 14)

if(NOT DEFINED EPONA_ROOT)
    set(EPONA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "Epona root directory.")
endif()

list(APPEND CMAKE_MODULE_PATH "${EPONA_ROOT}/Epona/cmake")

#Build flags
if (UNIX)
    if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-error=unused-command-line-argument")
    endif()

    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -pedantic")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s -O3")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -ggdb3 -O0")
elseif(WIN32)
    if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Werror -pedantic")
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -s -O3")
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g3 -ggdb3 -O0")
    elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP /W3")
        add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    endif()
endif()

#GLM
include(GlmConfig)
find_package(GLM)

set(EPONA_INCLUDE_DIR
    ${EPONA_ROOT}
    ${EPONA_ROOT}/Epona/include
    ${GLM_INCLUDE_DIR}
    CACHE LIST "Epona include directories."
)

if (NOT DEFINED INSTALL_INCLUDE_DIR)
    set(INSTALL_INCLUDE_DIR "include/godlike" CACHE STRING "Path to directory holding headers")
endif()

if (NOT DEFINED INSTALL_LIBRARY_DIR)
    set(INSTALL_LIBRARY_DIR "lib" CACHE STRING "Path to directory holding libraries")
endif()

set(EPONA_LIB ${EPONA_NAME} CACHE STRING "Name of Epona Math library")
set(EPONA_LIB_FULL ${EPONA_LIB}.dll CACHE STRING "Full name of Epona Math library")

set(EPONA_INSTALL_INCLUDE_DIR ${INSTALL_INCLUDE_DIR})
set(EPONA_INSTALL_LIBRARY_DIR ${INSTALL_LIBRARY_DIR}/${EPONA_NAME})

set(EPONA_VENDOR "Godlike")
set(EPONA_DESCRIPTION "Epona Math library")
set(EPONA_COMMENT "")
set(EPONA_COPYRIGHT "Copyright (C) 2017 by Godlike")
set(EPONA_LEGAL_TM "Distributed under the MIT license (http://opensource.org/licenses/MIT)")

set(EPONA_VERSION_MAJOR 0)
set(EPONA_VERSION_MINOR 1)
set(EPONA_VERSION_PATCH 0)
set(EPONA_VERSION_TWEAK 0)

set(EPONA_VERSION "${EPONA_VERSION_MAJOR}.${EPONA_VERSION_MINOR}.${EPONA_VERSION_PATCH}")
set(EPONA_SOVERSION "${EPONA_VERSION_MAJOR}.${EPONA_VERSION_MINOR}")

if (BUILD_SHARED_LIBS)
    add_definitions(-DEPONA_SHARED)
endif()
