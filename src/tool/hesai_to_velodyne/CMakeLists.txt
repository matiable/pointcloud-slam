cmake_minimum_required(VERSION 3.0.2)
project(hesai_to_velodyne)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -s")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pcl_ros
  pcl_conversions

  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS 
  DEPENDS PCL
)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
)

link_directories(
	include
	${PCL_LIBRARY_DIRS}
)


add_executable(hesai_to_velodyne src/hesai_to_velodyne.cpp)
add_dependencies(hesai_to_velodyne ${catkin_EXPORTED_TARGETS})
target_link_libraries(hesai_to_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

install(TARGETS hesai_to_velodyne
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
