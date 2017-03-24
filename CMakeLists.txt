cmake_minimum_required(VERSION 2.8.3)
project(bebop_teleop)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	std_msgs
)
find_package(SDL REQUIRED)

catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES bebop_teleop
#	CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#	DEPENDS system_lib ${SDL_LIBRARY}
)

catkin_package()

include_directories(
	include ${catkin_INCLUDE_DIRS} ${SDL_INCLUDE_DIR}
)

add_executable( bebop_teleop src/bebop_teleop.cpp )

target_link_libraries( bebop_teleop
	${catkin_LIBRARIES}
	${SDL_LIBRARY}
)
