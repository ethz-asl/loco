# Project configuration
cmake_minimum_required (VERSION 2.8)

project(locomotion_control)

include_directories ("include")

add_library(static_contact_force_distribution
  src/ContactForceDistribution.cpp
)