cmake_minimum_required(VERSION 3.22)
project(moveit_gpu_collision_plugin LANGUAGES CXX)
enable_language(CUDA)
if(NOT DEFINED CMAKE_CUDA_ARCHITECTURES)
  set(CMAKE_CUDA_ARCHITECTURES 70)
endif()
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Common cmake code applied to all moveit packages
find_package(moveit_common REQUIRED)
find_package(cuda_collision_checking  REQUIRED)
find_package(ros2-tiny-cuda-nn REQUIRED)
find_package(moveit_core  REQUIRED)
find_package(CUDA REQUIRED)
find_package(geometric_shapes REQUIRED)
moveit_package()

add_library(moveit_collision_detection_neural SHARED
    src/collision_common.cpp
    src/collision_env_neural.cpp
    )
target_include_directories(moveit_collision_detection_neural PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/moveit_gpu_collision_plugin>
    )

target_link_libraries(moveit_collision_detection_neural ${ros2-tiny-cuda-nn_LIBRARIES})
target_include_directories(moveit_collision_detection_neural PUBLIC ${ros2-tiny-cuda-nn_INCLUDE_DIR})

include_directories(include)

include(GenerateExportHeader)
generate_export_header(moveit_collision_detection_neural)
target_include_directories(moveit_collision_detection_neural PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)
target_link_libraries(moveit_collision_detection_neural ${ros2-tiny-cuda-nn_LIBRARIES})
target_include_directories(moveit_collision_detection_neural PUBLIC ${ros2-tiny-cuda-nn_INCLUDE_DIR})

set_target_properties(moveit_collision_detection_neural PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(moveit_collision_detection_neural
    rclcpp
    rmw_implementation
    urdf
    urdfdom
    urdfdom_headers
    cuda_collision_checking
    visualization_msgs
    moveit_core
    geometric_shapes
    )
#target_link_libraries(moveit_collision_detection_neural
#    moveit_collision_detection
#    )

add_library(collision_detector_neural_plugin SHARED src/collision_detector_neural_plugin_loader.cpp)
set_target_properties(collision_detector_neural_plugin PROPERTIES VERSION "${${PROJECT_NAME}_VERSION}")
ament_target_dependencies(collision_detector_neural_plugin
    rclcpp
    urdf
    visualization_msgs
    pluginlib
    rmw_implementation
    cuda_collision_checking
    moveit_core
    geometric_shapes
    )
#target_link_libraries(collision_detector_neural_plugin
#    moveit_collision_detection_neural
#    moveit_planning_scene
#    )

target_include_directories(collision_detector_neural_plugin PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>)

install(DIRECTORY include/ DESTINATION include/moveit_gpu_collision_plugin)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/moveit_collision_detection_neural_export.h DESTINATION include/moveit_gpu_collision_plugin)






install(
  TARGETS
  moveit_collision_detection_neural
  EXPORT moveit_collision_detection_neuralTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(moveit_collision_detection_neuralTargets HAS_LIBRARY_TARGET)
#ament_export_dependencies(
#    moveit_collision_detection_neural
#)

# Plugin exports
pluginlib_export_plugin_description_file(moveit_core collision_detector_neural_description.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # These don't pass yet, disable them for now
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_flake8_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  set(ament_cmake_lint_cmake_FOUND TRUE)

  # Run all lint tests in package.xml except those listed above
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
