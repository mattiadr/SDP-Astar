cmake_minimum_required(VERSION 3.22)
project(graph_generation)
project(sequential_astar)

set(CMAKE_CXX_STANDARD 20)

add_executable(graph_generation main.cpp)
add_executable(sequential_astar ../sequential_astar/main.cpp ../sequential_astar/main.cpp)
set(Boost_INCLUDE_DIR c:/SDK/boost_1_79_0/)
set(Boost_LIBRARY_DIR c:/SDK/boost_1_79_0/libs)
find_package(Boost 1.79.0 COMPONENTS REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
