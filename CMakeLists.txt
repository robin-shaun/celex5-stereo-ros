cmake_minimum_required(VERSION 2.8.3)
project(celex5_stereo)

find_package(catkin REQUIRED
  roscpp
  rospy
  std_msgs
  sensor_msgs
  celex5_msgs
  cv_bridge)
find_package(OpenCV REQUIRED)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES celex5_stereo
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)
include_directories(include ${OpenCV_INCLUDE_DIRS})


set(CeleX ./celex5dependencies/stereo)
set(CeleX_LIB CeleX)
set(CeleX_Driver_LIB CeleDriver)

include_directories(${CeleX}/include)
link_directories(${CeleX}/lib/Linux/x64)

add_executable(${PROJECT_NAME} src/celex5_stereo_ros.cpp)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES} 
  ${CeleX_LIB}
  ${CeleX_Driver_LIB}
)
