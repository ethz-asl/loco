# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)






################
### INCLUDES ###
################
set(TASK_INCL ${TASK_INCL} 

PARENT_SCOPE)

###############
### SOURCES ###
###############
set(TASK_SRCS ${TASK_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/ContactDetectorBase.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ContactDetectorConstantDuringStance.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/ContactDetectorFeedThrough.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 

PARENT_SCOPE)


set(USE_UTIL_XMLPARAMETERS ON CACHE BOOL "xml")
