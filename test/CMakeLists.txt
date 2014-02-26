# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

set(ROBOTMODEL_INCL  
)
set(UTILS_INCL  
)

add_definitions( -DSTANDALONE)
set(ARCH x86_64)
set(STANDALONE ON CACHE BOOL "standalone")
set(ISSUBDIRECTORY ON CACHE BOOL "issubdir")
add_subdirectory(../../starlethUtils starlethUtils EXCLUDE_FROM_ALL)
include_directories(${UTILS_INCL})
include_directories(${ROBOTMODEL_INCL})




add_subdirectory(foot_placement_strategy EXCLUDE_FROM_ALL)
add_subdirectory(gait_pattern EXCLUDE_FROM_ALL)
add_subdirectory(limb_coordinator EXCLUDE_FROM_ALL)
add_subdirectory(torso_control EXCLUDE_FROM_ALL)
add_subdirectory(locomotion_controller EXCLUDE_FROM_ALL)

