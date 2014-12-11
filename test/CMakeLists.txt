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

set(ROBOTMODEL_INCL  
)
set(UTILS_INCL  
)

add_definitions( -DSTANDALONE)
set(ARCH x86_64)
set(STANDALONE ON CACHE BOOL "standalone")
set(ISSUBDIRECTORY ON CACHE BOOL "issubdir")
add_subdirectory(../../robotUtils robotUtils EXCLUDE_FROM_ALL)
add_subdirectory(../../starlethUtils starlethUtils EXCLUDE_FROM_ALL)
include_directories(${UTILS_INCL})
include_directories(${ROBOTMODEL_INCL})




add_subdirectory(foot_placement_strategy EXCLUDE_FROM_ALL)
add_subdirectory(gait_pattern EXCLUDE_FROM_ALL)
add_subdirectory(limb_coordinator EXCLUDE_FROM_ALL)
add_subdirectory(torso_control EXCLUDE_FROM_ALL)
add_subdirectory(locomotion_controller EXCLUDE_FROM_ALL)

