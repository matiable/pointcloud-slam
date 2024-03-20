add_executable(run_mapping_online run_mapping_online.cc)
target_link_libraries(run_mapping_online
        ${PROJECT_NAME} gflags
        )
add_executable(run_mapping_offline run_mapping_offline.cc)
target_link_libraries(run_mapping_offline
        ${PROJECT_NAME} gflags
        )

install(TARGETS run_mapping_online run_mapping_offline jueying_lio
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
) 

install(DIRECTORY
  ../launch
  ../config
  ../rviz_cfg
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)