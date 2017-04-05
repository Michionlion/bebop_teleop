cmake_minimum_required(VERSION 2.8.3)
project(bebop_teleop)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	roscpp
	std_msgs
)

#find_package(SDL2 REQUIRED sdl2)
include(FindPkgConfig)
pkg_search_module(SDL2 REQUIRED sdl2)
#pkg_check_modules(SDL204 sdl2>=2.0.4)


catkin_package(
#	INCLUDE_DIRS include
#	LIBRARIES bebop_teleop
#	CATKIN_DEPENDS geometry_msgs roscpp std_msgs
#	DEPENDS system_lib ${SDL2_LIBRARY}
)

catkin_package()

include_directories(
	include ${SDL2_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS}
)

add_executable( bebop_teleop src/bebop_teleop.cpp )

target_link_libraries( bebop_teleop
	${SDL2_LIBRARIES}
	${catkin_LIBRARIES}
)
