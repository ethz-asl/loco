# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

find_package(OOQPEI REQUIRED)

################
### INCLUDES ###
################
set(LOCO_INCL ${LOCO_INCL}
	${OOQPEI_INCLUDE_DIRS}
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(LOCO_SRCS ${LOCO_SRCS}
    ${CMAKE_CURRENT_SOURCE_DIR}/ContactForceDistributionBase.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ContactForceDistribution.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(LOCO_LIBS ${LOCO_LIBS} 
	${OOQEI_LIBRARIES}
PARENT_SCOPE)

#############
### TESTS ###
#############
#add_subdirectory(test)


set(USE_UTIL_SCHWEIZERMESSER ON CACHE BOOL "schweizer-messer")