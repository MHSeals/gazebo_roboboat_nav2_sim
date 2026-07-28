cmake_minimum_required(VERSION 3.8)
project(roboboat_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf config rviz worlds scripts
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/gen_course.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # No copyright headers on config/asset files.
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
