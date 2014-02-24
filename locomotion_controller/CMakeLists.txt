# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

# CMake modules
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

find_package(Eigen REQUIRED)
find_package(Kindr REQUIRED)

################
### INCLUDES ###
################
set(TASK_INCL ${TASK_INCL} 
	${EIGEN_INCLUDE_DIRS}
	${Kindr_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/../temp_helpers/include
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(TASK_SRCS ${TASK_SRCS} 
	#${CMAKE_CURRENT_SOURCE_DIR}/src/FootPlacementStrategyBase.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 

PARENT_SCOPE)

add_subdirectory(test)

set(USE_UTIL_XMLPARAMETERS ON CACHE BOOL "xml")
