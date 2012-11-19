###############################################################################
# GTest setup
###############################################################################

macro(ecl_add_gtest TESTNAME)
  catkin_add_gtest(test_${TESTNAME} ${TESTNAME}.cpp)
  target_link_libraries(test_${TESTNAME} ${catkin_LIBRARIES})
ENDMACRO(ecl_add_gtest)

###############################################################################
# GTests
###############################################################################

ecl_add_gtest(grid_map)
ecl_add_gtest(grid_map_dynamic)

