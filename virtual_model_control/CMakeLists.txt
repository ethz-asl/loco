# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

include_directories ("include")

add_library(virtual_model_control
  src/VirtualModelController.cpp
)