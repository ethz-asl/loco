# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)


find_package(Eigen REQUIRED)
find_package(Kindr REQUIRED)

if (LOCO_VISUALIZER_SC)
    add_subdirectory(sc scVisualizer)
endif()

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
	${CMAKE_CURRENT_SOURCE_DIR}/VisualizerBase.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(LOCO_LIBS ${LOCO_LIBS} 

PARENT_SCOPE)

set(USE_UTIL_XMLPARAMETERS ON CACHE BOOL "xml")
set(USE_UTIL_POLYHARMONICSPLINES ON CACHE BOOL "polyharmonic splines")