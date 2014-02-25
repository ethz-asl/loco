# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Set Build Type
#   * None (CMAKE_C_FLAGS or CMAKE_CXX_FLAGS used)
#   * Debug (CMAKE_C_FLAGS_DEBUG or CMAKE_CXX_FLAGS_DEBUG)
#   * Release (CMAKE_C_FLAGS_RELEASE or CMAKE_CXX_FLAGS_RELEASE)
#   * RelWithDebInfo (CMAKE_C_FLAGS_RELWITHDEBINFO or CMAKE_CXX_FLAGS_RELWITHDEBINFO
#   * MinSizeRel (CMAKE_C_FLAGS_MINSIZEREL or CMAKE_CXX_FLAGS_MINSIZEREL) 
set(CMAKE_BUILD_TYPE Debug)

# CMake modules
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")


add_definitions(--std=c++11)

# Add CMake module path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if (BUILD_TEST)
set(CMAKE_CTEST_COMMAND ctest -V) 
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gtest/")
	message(STATUS "Building GTests!")
	option(BUILD_GTEST "build gtest" ON)
	add_subdirectory(gtest gtest EXCLUDE_FROM_ALL)
	enable_testing()

endif()
endif(BUILD_TEST)



# Add static contact force distribution
add_subdirectory(src/static_contact_force_distribution)

# Add virtual_model_control 
add_subdirectory(src/virtual_model_control)

# Add gait patern
add_subdirectory(src/gait_pattern)

# Add foot placement strategies
add_subdirectory(src/foot_placement_strategy)

# Add limb coordination
add_subdirectory(src/limb_coordinator)

# Add base control
add_subdirectory(src/base_control)

# Add base control
add_subdirectory(src/locomotion_controller)

# Add temporal helpers
add_subdirectory(src/temp_helpers)

if (BUILD_TEST AND BUILD_GTEST)
add_subdirectory(test EXCLUDE_FROM_ALL)
endif()

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
PARENT_SCOPE)

#################
### LIBRARIES ###
#################
set(TASK_LIBS ${TASK_LIBS} 
PARENT_SCOPE)

