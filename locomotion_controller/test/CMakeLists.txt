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

add_definitions( -DSTANDALONE)
set(ARCH x86_64)
set(STANDALONE ON CACHE BOOL "standalone")
set(ISSUBDIRECTORY ON CACHE BOOL "issubdir")

add_subdirectory(../../../starlethModel model)
add_subdirectory(../../../starlethUtils starlethUtils)


 

include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})
include_directories(../../thirdparty/tinyxml)
include_directories(../../limb_coordinator/include)
include_directories(../../gait_pattern/include)
include_directories(../../foot_placement_strategy/include)
include_directories(../../base_control/include)

#include_directories(${StarlETHRobotModel_INCLUDE_DIRS})
#include_directories(${StarlETHUtils_INCLUDE_DIRS})

set(LOCOMOTIONCONTROLLER_SRCS
	test_main.cpp
	LocomotionControllerTest.cpp
	
	../src/LocomotionControllerBase.cpp
	../src/LocomotionControllerDynamicGait.cpp
	
	../../foot_placement_strategy/src/FootPlacementStrategyBase.cpp
	../../foot_placement_strategy/src/FootPlacementStrategyInvertedPendulum.cpp
	
	../../limb_coordinator/src/LimbCoordinatorBase.cpp
	../../limb_coordinator/src/LimbCoordinatorDynamicGait.cpp
	
	../../gait_pattern/src/APS.cpp
	../../gait_pattern/src/GaitAPS.cpp
	../../gait_pattern/src/GaitPatternAPS.cpp
	
	../../thirdparty/tinyxml/tinystr.cpp
	../../thirdparty/tinyxml/tinyxml.cpp
	../../thirdparty/tinyxml/tinyxmlerror.cpp
	../../thirdparty/tinyxml/tinyxmlparser.cpp
)


add_executable( runUnitTestsLocomotionController EXCLUDE_FROM_ALL  ${LOCOMOTIONCONTROLLER_SRCS})
target_link_libraries(runUnitTestsLocomotionController  gtest_main gtest pthread ${StarlETHRobotModel_LIBRARIES} ${StarlETHUtils_LIBRARIES})
add_test( runUnitTestsLocomotionController ${EXECUTABLE_OUTPUT_PATH}/runUnitTestsLocomotionController )
add_dependencies(check runUnitTestsLocomotionController)