cmake_minimum_required(VERSION 3.10)

set(CMAKE_C_STANDARD 11)

project(plutovg C)

add_library(plutovg STATIC)

add_subdirectory(include)
add_subdirectory(source)
add_subdirectory(3rdparty/stb)
add_subdirectory(3rdparty/software)
