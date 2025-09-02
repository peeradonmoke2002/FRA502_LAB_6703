cmake_minimum_required(VERSION 3.8)
project(controller_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(dependencies
  std_msgs
)

file(
  GLOB_RECURSE INTERFACE_FILES
  RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
  ./srv/*.srv
)

rosidl_generate_interfaces(${PROJECT_NAME} 
	${INTERFACE_FILES}
DEPENDENCIES ${dependencies})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()