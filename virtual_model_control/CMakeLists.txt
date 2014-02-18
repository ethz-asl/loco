# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

include_directories ("include")

add_library(virtual_model_control
  src/VirtualModelController.cpp
)

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
	${CMAKE_CURRENT_SOURCE_DIR}/src/VirtualModelController.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 

PARENT_SCOPE)