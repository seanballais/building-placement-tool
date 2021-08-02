cmake_minimum_required(VERSION 3.13)

add_subdirectory(EAAssert/)
add_subdirectory(EABase/)
add_subdirectory(EAStdC/)
add_subdirectory(EAThread/)

add_subdirectory(imgui_impls/)

set(OpenGL_GL_PREFERENCE "GLVND")
option(BUILD_DEMOS "Build SDL_gpu demo programs" OFF)
add_subdirectory(sdl_gpu/)

set(BUILD_IMPLOT_SHARED OFF CACHE BOOL "Build implot shared library")
set(BUILD_IMPLOT_DEMO ON CACHE BOOL "Build implot demo")
add_subdirectory(implot/)

add_subdirectory(iprof/)
