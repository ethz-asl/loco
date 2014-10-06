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
	${CMAKE_CURRENT_SOURCE_DIR}/../../../robotBase/robotUtils/function_approximators/polyharmonicSplines
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(LOCO_SRCS ${LOCO_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/FootPlacementStrategyBase.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/FootPlacementStrategyInvertedPendulum.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/FootPlacementStrategyFreePlane.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/FootPlacementStrategyStaticGait.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(LOCO_LIBS ${LOCO_LIBS} 

PARENT_SCOPE)


set(USE_UTIL_POLYHARMONICSPLINES ON CACHE BOOL "polyharmonic splines")
