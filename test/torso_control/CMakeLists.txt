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


include_directories(${gtest_SOURCE_DIR}/include ${gtest_SOURCE_DIR})

include_directories(../../include)
#include_directories(../../thirdparty/tinyxml)



set(TORSOCONTROL_SRCS
	../test_main.cpp
	TorsoControlTest.cpp
	
	)
	set(ETasteaset
	../../src/common/TorsoBase.cpp
	../../src/common/TorsoStarlETH.cpp
	../../src/common/TorsoStateBase.cpp
	../../src/common/TorsoStateMeasured.cpp
	../../src/common/TorsoStateDesired.cpp
	../../src/common/TorsoPropertiesBase.cpp
	../../src/common/TorsoPropertiesStarlETH.cpp
	../../src/common/LegGroup.cpp
	../../src/common/LegBase.cpp
	../../src/common/LegStarlETH.cpp
	../../src/common/LegStateBase.cpp
	../../src/common/LegStateLiftOff.cpp
	../../src/common/LegStateTouchDown.cpp
	../../src/common/LegPropertiesBase.cpp
	../../src/common/LegPropertiesStarlETH.cpp
	
	../../src/torso_control/TorsoControlBase.cpp
	../../src/torso_control/TorsoControlDynamicGait.cpp
	../../src/torso_control/CoMOverSupportPolygonControl.cpp

)

# Add test cpp file
add_executable( runUnitTestsTorsoControl EXCLUDE_FROM_ALL  ${TORSOCONTROL_SRCS})
# Link test executable against gtest & gtest_main
target_link_libraries(runUnitTestsTorsoControl  gtest_main gtest pthread loco ${LOCO_LIBS})
add_test( runUnitTestsTorsoControl ${EXECUTABLE_OUTPUT_PATH}/runUnitTestsTorsoControl )
add_dependencies(check runUnitTestsTorsoControl)
