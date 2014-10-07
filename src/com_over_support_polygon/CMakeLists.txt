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
	${CMAKE_CURRENT_SOURCE_DIR}/CoMOverSupportPolygonControlBase.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CoMOverSupportPolygonControlDynamicGait.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CoMOverSupportPolygonControlStaticGait.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/CoMOverSupportPolygonControlLeverConfiguration.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(LOCO_LIBS ${LOCO_LIBS} 

PARENT_SCOPE)

set(USE_UTIL_XMLPARAMETERS ON CACHE BOOL "xml")
set(USE_UTIL_POLYHARMONICSPLINES ON CACHE BOOL "polyharmonic splines")