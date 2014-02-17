# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

include_directories ( "/include" )

file ( GLOB CODE
  ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
)

add_library(static_contact_force_distribution STATIC
  ${CODE}
)

file(GLOB TESTS_CODE ${CMAKE_CURRENT_SOURCE_DIR}/test/*.cpp) 
add_executable(static_contact_force_distribution_test ${TESTS_CODE} )
add_test(static_contact_force_distribution_test)