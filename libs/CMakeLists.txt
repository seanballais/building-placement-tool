cmake_minimum_required(VERSION 3.13)

add_subdirectory(imgui_impls/)

set(OpenGL_GL_PREFERENCE "GLVND")  # Make sure we use GLVND OpenGL drivers.
add_subdirectory(sdl_gpu/)

add_library(implot STATIC
    implot/implot.cpp
    implot/implot_items.cpp)
