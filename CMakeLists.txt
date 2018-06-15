cmake_minimum_required(VERSION 2.8.3)
project(crazyflie_rviz)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs cmake_modules nav_msgs tf)
#  geometry_msgs
#  nav_msgs
#  roscpp
#  tf
#	cmake_modules
#)
set(CMAKE_CXX_FLAGS "-std=c++11 -ggdb ${CMAKE_CXX_FLAGS}")
find_package(Eigen_REQUIRED)

catkin_package(
		INCLUDE_DIRS include ${Eigen_INCLUDE_DIRS}		
	
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  ${EIGEN_INCLUDE_DIR}
)

add_executable(crazyflie_rviz src/main.cpp)
target_link_libraries (crazyflie_rviz ${catkin_LIBRARIES})
