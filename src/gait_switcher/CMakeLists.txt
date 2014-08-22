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
	${CMAKE_CURRENT_SOURCE_DIR}/GaitSwitcherBase.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GaitSwitcherDynamicGaitDefault.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/GaitTransition.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(LOCO_LIBS ${LOCO_LIBS} 

PARENT_SCOPE)

