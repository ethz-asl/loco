
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
find_package(Kindr REQUIRED)

include_directories(${UTILS_INCL})
include_directories(${ROBOTMODEL_INCL})


include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Kindr_INCLUDE_DIRS})
include_directories(../../include)
include_directories(../../../robotUtils/include)
include_directories(${LOCO_INCL})

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})


set(FOOTPLACMENTSTRATEGY_SRCS
	../test_main.cpp
	FootPlacementStrategyTest.cpp
	
)

add_executable( runUnitTestsFootPlacementStrategy EXCLUDE_FROM_ALL ${FOOTPLACMENTSTRATEGY_SRCS})
target_link_libraries(runUnitTestsFootPlacementStrategy  gtest_main gtest pthread loco ${LOCO_LIBS} )
add_test( runUnitTestsFootPlacementStrategy ${EXECUTABLE_OUTPUT_PATH}/runUnitTestsFootPlacementStrategy )
add_dependencies(check runUnitTestsFootPlacementStrategy)


