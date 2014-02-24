
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



include_directories(${EIGEN_INCLUDE_DIRS})
include_directories(${Kindr_INCLUDE_DIRS})
include_directories(../include)
include_directories(../../temp_helpers/include)

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(../../thirdparty/tinyxml)


set(FOOTPLACMENTSTRATEGY_SRCS
	test_main.cpp
	FootPlacementStrategyTest.cpp
	../src/FootPlacementStrategyBase.cpp
	../src/FootPlacementStrategyInvertedPendulum.cpp
	../../thirdparty/tinyxml/tinystr.cpp
	../../thirdparty/tinyxml/tinyxml.cpp
	../../thirdparty/tinyxml/tinyxmlerror.cpp
	../../thirdparty/tinyxml/tinyxmlparser.cpp
)


add_executable( runUnitTestsFootPlacementStrategy EXCLUDE_FROM_ALL ${FOOTPLACMENTSTRATEGY_SRCS})
target_link_libraries(runUnitTestsFootPlacementStrategy  gtest_main gtest pthread starlethUtils starlethRobotModel )
add_test( runUnitTestsFootPlacementStrategy ${EXECUTABLE_OUTPUT_PATH}/runUnitTestsFootPlacementStrategy )
add_dependencies(check runUnitTestsFootPlacementStrategy)


