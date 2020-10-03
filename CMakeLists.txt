cmake_minimum_required(VERSION 3.13)
project(boyet-game)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

include("BuildType")

include("${CMAKE_BINARY_DIR}/conanbuildinfo.cmake")
conan_basic_setup()

set(CMAKE_CXX_STANDARD 17)

# We're gonna depend on CMake to locate SDL 2 in the system, since I can't
# compile SDL 2 successfully with the conan profile I have. In future, I will
# have to include the source code of SDL 2 so that I can ship the game with the
# SDL 2 library included.
INCLUDE(FindPkgConfig)

find_package(SDL2 REQUIRED)

include_directories(${SDL2_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/sdl_gpu/include)

add_subdirectory(libs/)
add_subdirectory(corex/)
add_subdirectory(bpt/)

target_link_libraries(bpt
    corex-core
    imgui-impls
    implot
    SDL_gpu
    ${CONAN_LIBS}
)

# Copy assets folder to the bin folder.
file(
    COPY assets
    DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
    PATTERN assets/raw EXCLUDE 
)

# Copy data folder to the bin folder.
file(
    COPY data
    DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

# Copy settings folder to the bin folder.
file(
    COPY settings
    DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

enable_testing()
