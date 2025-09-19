cmake_minimum_required(VERSION 3.8)
project(maps)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install map directories for both environments
install(DIRECTORY
  maps/real_env/
  DESTINATION share/${PROJECT_NAME}/maps/real_env
  OPTIONAL
)

install(DIRECTORY
  maps/simulation_env/
  DESTINATION share/${PROJECT_NAME}/maps/simulation_env
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()