cmake_minimum_required(VERSION 2.8)
set(CMAKE_BUILD_TYPE Release)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(PclMeshDec)

find_package(PCL 1.7 REQUIRED COMPONENTS common io)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(${PROJECT_NAME} "main.cpp")

target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
