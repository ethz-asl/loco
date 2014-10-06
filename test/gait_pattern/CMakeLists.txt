# Project configuration
cmake_minimum_required (VERSION 2.8)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
set(CMAKE_BUILD_TYPE Debug)

add_definitions(-std=c++0x)

find_package(Eigen REQUIRED)

include_directories(${EIGEN_INCLUDE_DIRS})

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

include_directories(../../include)
#include_directories(../../thirdparty/tinyxml)


set(GAITPATTERN_SRCS
	../test_main.cpp
	GaitPatternAPSTest.cpp
	#../../src/gait_pattern/APS.cpp
	#../../src/gait_pattern/GaitAPS.cpp
	#../../src/gait_pattern/GaitPatternAPS.cpp
	
)

# Add test cpp file
add_executable( runUnitTestsGaitPattern EXCLUDE_FROM_ALL ${GAITPATTERN_SRCS})
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTestsGaitPattern gtest_main gtest pthread loco ${LOCO_LIBS})
add_test( runUnitTestsGaitPattern ${EXECUTABLE_OUTPUT_PATH}/runUnitTestsGaitPattern )
add_dependencies(check runUnitTestsGaitPattern)