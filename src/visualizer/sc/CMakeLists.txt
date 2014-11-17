# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)


find_package(Eigen3 REQUIRED)
find_package(Kindr REQUIRED)

################
### INCLUDES ###
################
set(LOCO_INCL ${LOCO_INCL} 
	${EIGEN3_INCLUDE_DIR}
	${Kindr_INCLUDE_DIRS}
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(LOCO_SRCS ${LOCO_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/VisualizerSC.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GaitPatternAPSPreview.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GaitPatternFlightPhasesPreview.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(LOCO_LIBS ${LOCO_LIBS} 

PARENT_SCOPE)
