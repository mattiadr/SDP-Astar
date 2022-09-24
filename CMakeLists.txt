cmake_minimum_required(VERSION 3.21)
project(graph_generation)
project(sequential_astar)

set(CMAKE_CXX_STANDARD 20)

add_executable(graph_generation graph_generation/main.cpp)
add_executable(sequential_astar sequential_astar/main.cpp)
set(Boost_INCLUDE_DIR d:/Applications/boost_1_80_0/)
set(Boost_LIBRARY_DIR d:/Applications/boost_1_80_0/libs)
find_package(Boost 1.80.0 COMPONENTS REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
