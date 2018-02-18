# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required(VERSION 3.1)

project(Epona)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

option(EPONA_BUILD_DEMOS "Build Epona demo." ON)
option(EPONA_BUILD_TESTS "Build Epona tests." ON)

message(STATUS "${PROJECT_NAME} configuration:")
message(STATUS "-- EPONA_BUILD_DEMOS: ${EPONA_BUILD_DEMOS}")
message(STATUS "-- EPONA_BUILD_TESTS: ${EPONA_BUILD_DEMOS}")

set(EPONA_ROOT "${CMAKE_CURRENT_SOURCE_DIR}" CACHE STRING "Epona root directory.")

include(EponaConfig)

if (NOT CMAKE_ARCHIVE_OUTPUT_DIRECTORY)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
endif()

if (NOT CMAKE_LIBRARY_OUTPUT_DIRECTORY)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output/lib)
endif()

if (NOT CMAKE_RUNTIME_OUTPUT_DIRECTORY)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output)
endif()

add_subdirectory(Epona)

if (${EPONA_BUILD_DEMOS})
    add_subdirectory(demo)
endif()

if (EPONA_BUILD_TESTS)
    enable_testing()
    set(CATCH_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/third_party/Catch2)
    add_subdirectory(test)
endif()

get_directory_property(HAS_PARENT PARENT_DIRECTORY)
if (HAS_PARENT)
    set(EPONA_MATH_LIB ${EPONA_LIB} PARENT_SCOPE)
endif()
