cmake_minimum_required(VERSION 3.13)

add_subdirectory(corex/)

add_test(
    NAME corex-core-test
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    COMMAND ${CMAKE_BINARY_DIR}/bin/tests/corex-core-test
)
