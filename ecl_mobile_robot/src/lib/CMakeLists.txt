###############################################################################
# SOURCES
###############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

###############################################################################
# INSTALL
###############################################################################

rosbuild_add_library(ecl_mobile_robot ${SOURCES})
install(TARGETS ecl_mobile_robot LIBRARY DESTINATION lib PERMISSIONS OWNER_READ OWNER_WRITE OWNER_EXECUTE GROUP_READ GROUP_EXECUTE WORLD_READ WORLD_EXECUTE CONFIGURATIONS Debug Release RelWithDebInfo) 
