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
	../../../robotBase/robotUtils/function_approximators/dynamicMovementPrimitive
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(LOCO_SRCS ${LOCO_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/TorsoControlBase.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/TorsoControlDynamicGait.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/TorsoControlDynamicGaitFreePlane.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/TorsoControlStaticGait.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/TorsoControlJump.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(LOCO_LIBS ${LOCO_LIBS} 

PARENT_SCOPE)

set(USE_UTIL_XMLPARAMETERS ON CACHE BOOL "xml")
set(USE_UTIL_POLYHARMONICSPLINES ON CACHE BOOL "polyharmonic splines")
set(USE_UTIL_DYNAMICMOVEMENTPRIMITIVE ON CACHE BOOL "dynamic movement primitive")
