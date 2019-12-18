cmake_minimum_required(VERSION 3.5)
project(astar-cpp-new)

set(CMAKE_CXX_STANDARD 11)

include_directories(include)
link_directories(lib/x86-64_linux/static_pic)

add_definitions(-DIL_STD)
add_definitions(-DCPX_FORCE_DEPRECATION_WARNINGS)

file(GLOB model RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/model/*.cpp include/model/*.h)

file(GLOB aStar RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/aStar/*.cpp include/aStar/*.h include/aStar/*.hpp)

file(GLOB simplifier RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/pathSimplifier/*.cpp include/pathSimplifier/*.h)

file(GLOB generator RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/wayPointsGenerator/*.cpp include/wayPointsGenerator/*.hpp)

#file(GLOB tree2grid RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
#        include/octree2ThreeDModel/*.cpp include/octree2ThreeDModel/*.hpp)

file(GLOB HGJPathSmoothFILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
        include/HGJPathSmooth/*.cpp include/HGJPathSmooth/*.h)

add_library(HGJ_Path_Smooth ${HGJPathSmoothFILES})

#find_package(PCL 1.8 REQUIRED)
#include_directories(${PCL_INCLUDE_DIRS})
#link_directories(${PCL_LIBRARY_DIRS})
#add_definitions(${PCL_DEFINITIONS})
#
#find_package(Boost REQUIRED COMPONENTS system thread)
#include_directories( include ${Boost_INCLUDE_DIRS})
#link_libraries( ${Boost_LIBRARIES})

add_executable(astar-cpp main.cpp ${model} ${aStar} ${simplifier} ${generator}) #${PCL_LIBRARIES})

target_link_libraries(HGJ_Path_Smooth ilocplex cplex concert m pthread dl)
target_link_libraries(astar-cpp ilocplex cplex concert m pthread dl HGJ_Path_Smooth)
