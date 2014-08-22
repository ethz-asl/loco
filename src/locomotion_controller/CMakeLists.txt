# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)



find_package(Eigen REQUIRED)
find_package(Kindr REQUIRED)

################
### INCLUDES ###
################
set(LOCO_INCL ${LOCO_INCL} 
	${EIGEN_INCLUDE_DIRS}
	${Kindr_INCLUDE_DIRS}
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(LOCO_SRCS ${LOCO_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/LocomotionControllerBase.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LocomotionControllerDynamicGait.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/LocomotionControllerDynamicGaitDefault.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(LOCO_LIBS ${LOCO_LIBS} 

PARENT_SCOPE)

#add_subdirectory(test EXCLUDE_FROM_ALL)

set(USE_UTIL_XMLPARAMETERS ON CACHE BOOL "xml")
