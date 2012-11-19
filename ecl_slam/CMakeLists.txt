##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(ecl_slam)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
    ecl_exceptions
    ecl_time
    ecl_linear_algebra
    )
catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS
        ecl_exceptions
        ecl_time
        ecl_linear_algebra
        )
include_directories(SYSTEM include ${catkin_INCLUDE_DIRS})

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(include)



