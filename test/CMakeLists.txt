# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

if(BUILD_TESTING)
message(STATUS "LOCOMOTION CONTROL")
if(NOT TARGET starlethUtils)


message(STATUS "LOCOMOTION CONTROL: starlethUtils")

set(ROBOTMODEL_INCL  
)
set(UTILS_INCL  
)

add_definitions( -DSTANDALONE)
set(ARCH x86_64)
set(STANDALONE ON CACHE BOOL "standalone")
set(ISSUBDIRECTORY ON CACHE BOOL "issubdir")
add_subdirectory(../../starlethUtils starlethUtils)
include_directories(${UTILS_INCL})
include_directories(${ROBOTMODEL_INCL})
endif(NOT TARGET starlethUtils)
endif(BUILD_TESTING	)


# Add foot placement strategies
add_subdirectory(foot_placement_strategy EXCLUDE_FROM_ALL)

