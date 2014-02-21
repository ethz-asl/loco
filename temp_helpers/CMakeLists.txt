# Project configuration
cmake_minimum_required (VERSION 2.8)

project(temp_helpers)

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
	${CMAKE_CURRENT_SOURCE_DIR}/src/Legs.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 

PARENT_SCOPE)