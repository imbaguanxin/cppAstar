cmake_minimum_required(VERSION 3.14)
project(astar-cpp)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)


file(GLOB modelFiles RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/model/*.cpp include/model/*.h)

file(GLOB aStar RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/aStar/*.cpp include/model/*.h)

add_executable(cppPractice main.cpp ${modelFiles} ${aStar})