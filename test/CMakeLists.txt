# Copyright (C) 2017 by Godlike
# This code is licensed under the MIT license (MIT)
# (http://opensource.org/licenses/MIT)

cmake_minimum_required(VERSION 3.0)
cmake_policy(VERSION 3.0)

project(EponaTests)

include_directories(${EPONA_INCLUDE_DIR} ${CATCH_ROOT}/single_include)

function(epona_add_test)
    set(options)
    set(oneValueArgs NAME)
    set(multiValueArgs SOURCE ARGS DEPENDS)
    cmake_parse_arguments(
        epona_add_test
        "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGN}
    )

    add_executable(
        ${epona_add_test_NAME}_test
        ${epona_add_test_SOURCE}
    )

    if (epona_add_test_DEPENDS)
        target_link_libraries(${epona_add_test_NAME}_test ${epona_add_test_DEPENDS})
    endif()

    add_test(
        NAME ${epona_add_test_NAME}
        COMMAND ${epona_add_test_NAME}_test ${epona_add_test_ARGS}
    )
endfunction()

epona_add_test(NAME Collision
    SOURCE AnalysisTests.cpp
    DEPENDS ${EPONA_LIB}
)
