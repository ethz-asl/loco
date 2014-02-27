# Project configuration
cmake_minimum_required (VERSION 2.8)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Set Build Type
#   * None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
#   * Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
#   * Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
#   * RelWithDebInfo (CMAKE_C_FLAGS_RELWITHDEBINFO or CMAKE_CXX_FLAGS_RELWITHDEBINFO
#   * MinSizeRel (CMAKE_C_FLAGS_MINSIZEREL or CMAKE_CXX_FLAGS_MINSIZEREL) 
set(CMAKE_BUILD_TYPE Debug)

# CMake modules
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")

add_definitions(--std=c++11)


# Add static contact force distribution
add_subdirectory(../../src/contact_force_distribution contact_force_distribution)

# Add virtual model control 
add_subdirectory(../../src/virtual_model_control virtual_model_control)

# Add gait patern
add_subdirectory(../../src/gait_pattern gait_pattern)

# Add foot placement strategies
add_subdirectory(../../src/foot_placement_strategy foot_placement_strategy)

# Add limb coordination
add_subdirectory(../../src/limb_coordinator limb_coordinator)

# Add base control
add_subdirectory(../../src/torso_control torso_control)

add_subdirectory(../../src/common common)


add_subdirectory(../../src/locomotion_controller locomotion_controller)

# Add temporal helpers
add_subdirectory(../../src/temp_helpers temp_helpers)


set(TASK_INCL ${TASK_INCL}
	${CMAKE_CURRENT_SOURCE_DIR}/include 
)

################
### INCLUDES ###
################
set(TASK_INCL ${TASK_INCL}

PARENT_SCOPE)

###############
### SOURCES ###
###############
set(TASK_SRCS ${TASK_SRCS}
	${CMAKE_CURRENT_SOURCE_DIR}/src/LocoExample_Task.cpp 
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 
PARENT_SCOPE)