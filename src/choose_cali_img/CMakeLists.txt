cmake_minimum_required(VERSION 2.8.3)
project(choose_cali_img)
SET(CMAKE_BUILD_TYPE Debug)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -ggdb")
find_package(OpenCV REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})
add_executable(choose_cali_img src/main.cpp)
target_link_libraries(choose_cali_img ${OpenCV_LIBS})

