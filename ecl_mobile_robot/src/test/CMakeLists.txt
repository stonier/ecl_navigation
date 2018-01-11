###############################################################################
# GTest setup
###############################################################################

macro(ecl_add_gtest TESTNAME)
  catkin_add_gtest(ecl_test_${TESTNAME} ${TESTNAME}.cpp)
  if(TARGET ecl_test_${TESTNAME})
    target_link_libraries(ecl_test_${TESTNAME} ${PROJECT_NAME} 
      ${ecl_build_LIBRARIES}
      ${ecl_errors_LIBRARIES}
      ${ecl_geometry_LIBRARIES}
      ${ecl_math_LIBRARIES}
      ${ecl_formatters_LIBRARIES}
      ${ecl_linear_algebra_LIBRARIES}
	)
  endif()
endmacro(ecl_add_gtest)

###############################################################################
# GTests
###############################################################################

ecl_add_gtest(partial_inverse)

