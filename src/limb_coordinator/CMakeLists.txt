# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

find_package(Eigen REQUIRED)
find_package(Kindr REQUIRED)

################
### INCLUDES ###
################
set(TASK_INCL ${TASK_INCL} 
	${EIGEN_INCLUDE_DIRS}
	${Kindr_INCLUDE_DIRS}
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(TASK_SRCS ${TASK_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/LimbCoordinatorBase.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LimbCoordinatorDynamicGait.cpp
		${CMAKE_CURRENT_SOURCE_DIR}/LimbCoordinatorJump.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 

PARENT_SCOPE)


set(USE_UTIL_XMLPARAMETERS ON CACHE BOOL "xml")

