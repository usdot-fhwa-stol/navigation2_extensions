cmake_minimum_required(VERSION 3.22)
project(carma_nav2_port_drayage_demo)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(carma_nav2_port_drayage_demo
  src/port_drayage_demo.cpp
)

ament_auto_add_executable(carma_nav2_port_drayage_demo_node
  src/port_drayage_demo_node.cpp
)

ament_auto_package()
