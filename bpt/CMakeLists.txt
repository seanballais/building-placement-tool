cmake_minimum_required(VERSION 3.13)

# TODO: Replace this with a target_include_directories.
include_directories("${CMAKE_SOURCE_DIR}/corex/src/")
include_directories("${CMAKE_CURRENT_SOURCE_DIR}/src/")

add_subdirectory(src/)
