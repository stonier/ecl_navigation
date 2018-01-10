##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 3.5)
project(ecl_mobile_robot)

##############################################################################
# Catkin
##############################################################################

find_package(ament_cmake REQUIRED)
find_package(ecl_build  REQUIRED)
find_package(ecl_errors REQUIRED)
#find_package(ecl_geometry# REQUIRED)
find_package(ecl_math REQUIRED)
find_package(ecl_formatters REQUIRED)
#find_package(ecl_linear_algebra REQUIRED)

    
##############################################################################
# Project Configuration
##############################################################################

ecl_enable_cxx11_compiler()
include_directories(include 
  ${ecl_build_INCLUDE_DIRS}
  ${ecl_errors_INCLUDE_DIRS}
  ${ecl_geometry_INCLUDE_DIRS}
  ${ecl_math_INCLUDE_DIRS}
  ${ecl_formatters_INCLUDE_DIRS}
  ${ecl_linear_algebra_INCLUDE_DIRS}
  )

##############################################################################
# Sources
##############################################################################

add_subdirectory(include)
add_subdirectory(src)



