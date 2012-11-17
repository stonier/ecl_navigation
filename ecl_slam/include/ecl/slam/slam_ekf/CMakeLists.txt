###############################################################################
# FILES
###############################################################################

FILE(GLOB HEADERS RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.hpp)

INSTALL(FILES ${HEADERS} DESTINATION ${ECL_DIR_HEADERS}/slam/slam_ekf COMPONENT development)
