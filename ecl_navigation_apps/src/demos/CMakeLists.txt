###############################################################################
# Demos
###############################################################################

add_executable(demo_slammer1d slammer1d.cpp)
target_link_libraries(demo_slammer1d ${catkin_LIBRARIES})
install(TARGETS demo_slammer1d RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

