# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

find_package(OOQPEI REQUIRED)

################
### INCLUDES ###
################
set(TASK_INCL ${TASK_INCL}
	${OOQPEI_INCLUDE_DIRS}
	${CMAKE_CURRENT_SOURCE_DIR}/include
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(TASK_SRCS ${TASK_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/src/ContactForceDistribution.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 
	${OOQEI_LIBRARIES}
PARENT_SCOPE)

#############
### TESTS ###
#############
#add_subdirectory(test)