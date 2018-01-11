###############################################################################
# Files
###############################################################################

add_executable(demo_partial_inverse partial_inverse.cpp)
target_link_libraries(demo_partial_inverse ${PROJECT_NAME}
  ${ecl_build_LIBRARIES}
  ${ecl_errors_LIBRARIES}
  ${ecl_geometry_LIBRARIES}
  ${ecl_math_LIBRARIES}
  ${ecl_formatters_LIBRARIES}
  ${ecl_linear_algebra_LIBRARIES}
)
install(TARGETS demo_partial_inverse RUNTIME DESTINATION bin)
