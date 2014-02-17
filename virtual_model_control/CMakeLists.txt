# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

include_directories ( "/include" )

file ( GLOB CODE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(virtual_model_control STATIC
  ${CODE}
)

file(GLOB TESTS_CODE ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp) 
add_executable(virtual_model_control_test ${TESTS_CODE} )
add_test(virtual_model_control_test)