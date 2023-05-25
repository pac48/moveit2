add_library(moveit_collision_detection_neural SHARED
  src/collision_common.cpp
  src/collision_env_neural.cpp
)
target_include_directories(moveit_collision_detection_neural PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/moveit_core>
)
include(GenerateExportHeader)
generate_export_header(moveit_collision_detection_neural)
target_include_directories(moveit_collision_detection_neural PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
set_target_properties(moveit_collision_detection_neural PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_collision_detection_neural
  rclcpp
  rmw_implementation
  urdf
  urdfdom
  urdfdom_headers
  LIBFCL
  visualization_msgs
)
target_link_libraries(moveit_collision_detection_neural
  moveit_collision_detection
)

add_library(collision_detector_neural_plugin SHARED src/collision_detector_neural_plugin_loader.cpp)
set_target_properties(collision_detector_neural_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(collision_detector_neural_plugin
  rclcpp
  urdf
  visualization_msgs
  pluginlib
  rmw_implementation
)
target_link_libraries(collision_detector_neural_plugin
  moveit_collision_detection_neural
  moveit_planning_scene
)

install(DIRECTORY include/ DESTINATION include/moveit_core)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_collision_detection_neural_export.h DESTINATION include/moveit_core)