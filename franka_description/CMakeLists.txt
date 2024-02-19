cmake_minimum_required(VERSION 3.5)
project(franka_description)
find_package(ament_cmake REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_urdf_tests test/urdf_tests.py)

endif()

# Install launch files.
install(DIRECTORY
  launch robots meshes rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/${PROJECT_NAME}.sh.in"
)
ament_package()