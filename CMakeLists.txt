############################################################################################
# Software License Agreement (BSD License)
#
# Copyright (c) 2014, Christian Gehring, Péter Fankhauser, C. Dario Bellicoso, Stelian Coros
# All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of Autonomous Systems Lab nor ETH Zurich
#     nor the names of its contributors may be used to endorse or
#     promote products derived from this software without specific
#     prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
#
# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

message("      Including locomotion controller")


#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# CMake modules
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../cmake")

add_definitions(--std=c++11)

find_package(OOQPEI REQUIRED)

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
add_subdirectory(src/state_switcher)
add_subdirectory(src/com_over_support_polygon)


set(LOCO_INCL ${LOCO_INCL}
	${CMAKE_CURRENT_SOURCE_DIR}/include
	$ENV{LAB_ROOT}/starlethUtils/ros/ros_srv
)


include_directories($ENV{LAB_ROOT}/robotUtils/include)
include_directories($ENV{LAB_ROOT}/starlethModel/include)
include_directories($ENV{LAB_ROOT}/starlethUtils/ros/ros_srv)
include_directories(${LOCO_INCL}) 
include_directories(${TASK_INCL})


set(LOCO_LIBS ${LOCO_LIBS}
  starlethRobotModel robotUtils tinyxml ${OOQEI_LIBRARIES}
)


# Add include directories to task
set(TASK_INCL ${TASK_INCL} ${LOCO_INCL}
PARENT_SCOPE)


# Add this library to the task
set(TASK_LIBS ${TASK_LIBS} ${LOCO_LIBS}
    loco   
PARENT_SCOPE)


add_library(loco STATIC ${LOCO_SRCS})
target_link_libraries(loco ${LOCO_LIBS})

# Add Doxygen documentation
if (BUILD_DOC)
add_subdirectory(doc/doxygen)
endif (BUILD_DOC)


if (BUILD_TEST AND BUILD_GTEST)
add_subdirectory(test EXCLUDE_FROM_ALL)
endif()





	