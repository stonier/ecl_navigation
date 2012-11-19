##############################################################################
# Cmake
##############################################################################

cmake_minimum_required(VERSION 2.8.3)
project(ecl_navigation_apps)

##############################################################################
# Catkin
##############################################################################

find_package(catkin REQUIRED COMPONENTS 
    ecl_formatters
    ecl_slam
    ecl_utilities
    )
catkin_package()

include_directories(SYSTEM include ${catkin_INCLUDE_DIRS})

##############################################################################
# Actual project configuration
##############################################################################

add_subdirectory(src)



