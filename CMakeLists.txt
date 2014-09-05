# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)


set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# CMake modules
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

add_definitions(--std=c++11)

# Add CMake module path
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

if (BUILD_TEST)
set(CMAKE_CTEST_COMMAND ctest)
#set(CMAKE_CTEST_COMMAND ctest -V) # Detailed output
add_custom_target(check COMMAND ${CMAKE_CTEST_COMMAND})

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/gtest/")
	message(STATUS "Building GTests!")
	option(BUILD_GTEST "build gtest" ON)
	add_subdirectory(gtest gtest EXCLUDE_FROM_ALL)
	enable_testing()

endif()
endif(BUILD_TEST)


add_subdirectory(src/common)
add_subdirectory(src/contact_force_distribution)
add_subdirectory(src/motion_control)
add_subdirectory(src/gait_pattern)
add_subdirectory(src/foot_placement_strategy)
add_subdirectory(src/limb_coordinator)
add_subdirectory(src/torso_control)
add_subdirectory(src/mission_control)
add_subdirectory(src/terrain_perception)
add_subdirectory(src/visualizer)
add_subdirectory(src/tools)
add_subdirectory(src/contact_detection)
add_subdirectory(src/gait_switcher)
add_subdirectory(src/locomotion_controller)
add_subdirectory(src/event_detection)


set(LOCO_INCL ${LOCO_INCL}
	${CMAKE_CURRENT_SOURCE_DIR}/include 
)

include_directories(../robotUtils/include)
include_directories(${LOCO_INCL}) 
include_directories(${TASK_INCL})

set(LOCO_LIBS ${LOCO_LIBS}
  starlethRobotModel robotUtils starlethUtils tinyxml 
)


# Add include directories to task
set(TASK_INCL ${TASK_INCL} ${LOCO_INCL}
PARENT_SCOPE)


# Add this library to the task
set(TASK_LIBS ${TASK_LIBS} ${LOCO_LIBS} 
    loco  
PARENT_SCOPE)


add_library(loco ${LOCO_SRCS})
target_link_libraries(loco ${LOCO_LIBS})

# Add Doxygen documentation
if (BUILD_DOC)
add_subdirectory(doc/doxygen)
endif (BUILD_DOC)


if (BUILD_TEST AND BUILD_GTEST)
add_subdirectory(test EXCLUDE_FROM_ALL)
endif()