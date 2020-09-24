cmake_minimum_required(VERSION 3.13)

add_executable(corex-core-test
    test_main.cpp
)

target_link_libraries(corex-core-test
    corex-core
    ${CONAN_LIBS}
)
