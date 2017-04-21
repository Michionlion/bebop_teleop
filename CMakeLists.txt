cmake_minimum_required(VERSION 2.8.3)
project(bebop_teleop)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	std_msgs
	sensor_msgs
	cv_bridge
	image_transport
)

include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)


catkin_package(
	INCLUDE_DIRS include
	LIBRARIES bebop_teleop
	CATKIN_DEPENDS geometry_msgs roscpp std_msgs
	DEPENDS ${SDL2_LIBRARY}
)
include_directories(include ${SDL2_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable( bebop_teleop src/Events.cpp src/Input.cpp src/Window.cpp src/ManualControl.cpp src/main.cpp)
add_executable( webcam src/WebcamPublisher.cpp)
target_link_libraries( bebop_teleop
	${SDL2_LIBRARIES}
	${catkin_LIBRARIES}
)

target_link_libraries( webcam
	${catkin_LIBRARIES}
)
