#-------------------------------------------------------------------------------
#
# Sub-project CMake file for the SL software
#
#  Rough terrain locomotion for StarlETH
#
# Autonomous Systems Lab
# ETH Zurich
#
# Author(s): Peter Fankhauser
# email    : pfankhauser@ethz.ch
# Date     : 8-Jul-2013
#
#-------------------------------------------------------------------------------
cmake_minimum_required (VERSION 2.6)  
message("      Including task rough terrain")

################
### INCLUDES ###
################
set(TASK_INCL ${TASK_INCL} 
	${CMAKE_CURRENT_SOURCE_DIR} 
PARENT_SCOPE)

###############
### SOURCES ###
###############
set(TASK_SRCS ${TASK_SRCS} 
	${CMAKE_CURRENT_SOURCE_DIR}/RoughTerrain_Task.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/VirtualModelController.cpp
	${CMAKE_CURRENT_SOURCE_DIR}/ContactForceDistribution.cpp
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 

PARENT_SCOPE)

#################
### UTILITIES ###
#################
set(USE_UTIL_TASKTIMER ON CACHE BOOL "task timer")
set(USE_UTIL_DISTURBROBOT ON CACHE BOOL "disturb robot")
set(USE_UTIL_DRAWARROW ON CACHE BOOL "draw arrow")
set(USE_UTIL_DRAWSPHERE ON CACHE BOOL "draw sphere")
set(USE_UTIL_DRAWFRAME ON CACHE BOOL "draw frame")
set(USE_UTIL_DRAWGHOST ON CACHE BOOL "draw ghost")