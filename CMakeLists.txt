cmake_minimum_required(VERSION 3.13)
project(bpt)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/")

include("BuildType")

include("${CMAKE_BINARY_DIR}/conanbuildinfo.cmake")
conan_basic_setup()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Mhmm. We're sticking with just Clang. Note that -stdlib is a Clang-only flag.
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -stdlib=libc++")

# Make sure no optimizations are present during a debug build. This will let us
# get accurate profiling results.
if (CMAKE_BUILD_TYPE STREQUAL Debug)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
endif()

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/EAStdC/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/sdl_gpu/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/libs/pcg-cpp/include)

add_subdirectory(libs/)
add_subdirectory(corex/)
add_subdirectory(bpt/)

target_link_libraries(bpt
    iprof
    corex-core
    imgui-impls
    implot
    SDL_gpu
    EAStdC
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
