cmake_minimum_required(VERSION 3.5)
project(astar-cpp)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)

file(GLOB model RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/model/*.cpp include/model/*.h)

file(GLOB aStar RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/aStar/*.cpp include/model/*.h)

file(GLOB simplifier RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/pathSimplifier/*.cpp include/pathSimplifier/*.h)

file(GLOB generator RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/wayPointsGenerator/*.cpp include/wayPointsGenerator/*.h)

add_executable(astar-cpp main.cpp ${model} ${aStar} ${simplifier} ${generator})