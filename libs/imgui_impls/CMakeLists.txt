cmake_minimum_required(VERSION 3.13)

add_library(imgui-impls STATIC
    imgui_impl_opengl3.cpp
    imgui_impl_sdl.cpp
)
