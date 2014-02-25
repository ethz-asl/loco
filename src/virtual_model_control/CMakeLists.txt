# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

################
### INCLUDES ###
################
set(TASK_INCL ${TASK_INCL} 
	${CMAKE_CURRENT_SOURCE_DIR}/include
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(TASK_SRCS ${TASK_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/VirtualModelController.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 

PARENT_SCOPE)

#############
### TESTS ###
#############
#add_subdirectory(test)