cmake_minimum_required(VERSION 3.13)

add_executable(bpt
    utils.cpp
    Application.cpp
    MainScene.cpp
    GA.cpp
    GD.cpp
    GWO.cpp
    gui.hpp
    generator.cpp
    evaluator.cpp
    HC.cpp
    operators.cpp
    SA.cpp
    ds/AlgorithmType.hpp
    ds/GAResult.cpp
    ds/GWOResult.cpp
    ds/Result.cpp
    ds/Solution.cpp
    ds/Time.cpp)
