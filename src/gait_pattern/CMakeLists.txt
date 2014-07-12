# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

################
### INCLUDES ###
################
set(LOCO_INCL ${LOCO_INCL} 

PARENT_SCOPE)

###############
### SOURCES ###
###############
set(LOCO_SRCS ${LOCO_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/APS.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GaitAPS.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GaitPatternAPS.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GaitPatternFlightPhases.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(LOCO_LIBS ${LOCO_LIBS} 

PARENT_SCOPE)


set(USE_UTIL_XMLPARAMETERS ON CACHE BOOL "xml")
