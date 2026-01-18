cmake_minimum_required(VERSION 3.26.3)
project(manager)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")
include_directories(include)
include_directories(../GridMap2D/include)
include_directories(./path_searching/include/path_searching)

add_library( manager STATIC
    src/manager.cpp
    )
