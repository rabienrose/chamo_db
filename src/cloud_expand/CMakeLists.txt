cmake_minimum_required(VERSION 2.8.3)
project(cloud_expand)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -ggdb")

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
)

find_package(OpenCV REQUIRED)
message(STATUS ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(cloud_expand src/main.cpp)
target_link_libraries(cloud_expand ${catkin_LIBRARIES} ${OpenCV_LIBS})
set(CATKIN_PACKAGE_BIN_DESTINATION bin/)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
 )


