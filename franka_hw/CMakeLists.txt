cmake_minimum_required(VERSION 3.5)
project(franka_hw)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_library(franka_hw src/franka_hw.cpp)
target_include_directories(franka_hw PRIVATE include)
ament_target_dependencies(franka_hw
  hardware_interface
  rclcpp
)
install(TARGETS franka_hw
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
)
install(DIRECTORY include/
  DESTINATION include
)
ament_package()
