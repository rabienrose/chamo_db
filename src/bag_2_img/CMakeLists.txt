cmake_minimum_required(VERSION 2.8.3)
project(bag_2_img)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -ggdb")

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/src)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
  rosbag
  cv_bridge
)

find_package(OpenCV REQUIRED)
message(STATUS ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(bag_2_img src/main.cpp)
target_link_libraries(bag_2_img ${catkin_LIBRARIES} ${OpenCV_LIBS})
set(CATKIN_PACKAGE_BIN_DESTINATION bin/)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
 )


