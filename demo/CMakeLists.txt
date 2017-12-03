# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required (VERSION 3.1)

project(EponaDemo)

#Target
include(EponaConfig)
include_directories (
    ${EPONA_INCLUDE_DIR}
)
set(DEMO_HEADERS
)
set(DEMO_SOURCES
    Main.cpp
)

add_executable (${PROJECT_NAME}
    ${DEMO_SOURCES}
    ${DEMO_HEADERS}
)
target_link_libraries(${PROJECT_NAME}
    ${EPONA_LIB}
)
