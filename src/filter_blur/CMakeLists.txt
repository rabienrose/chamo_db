cmake_minimum_required(VERSION 2.8.3)
project(filter_blur)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -ggdb")
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
add_executable(filter_blur src/main.cpp)
target_link_libraries(filter_blur ${OpenCV_LIBS})

