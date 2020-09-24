# Specifies the build type of this project.
# The default build type of this project is "debug", since debug compilations
# will be more frequent than release compilations.

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

if (CMAKE_BUILD_TYPE STREQUAL Debug)
    add_compile_definitions(DEBUG)
endif()

message(STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
