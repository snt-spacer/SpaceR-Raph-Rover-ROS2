cmake_minimum_required(VERSION 3.8)
project(bumblebee_publisher)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Spinnaker C++ SDK installed by the Spinnaker .deb packages
set(SPINNAKER_INCLUDE_DIR /opt/spinnaker/include)
find_library(SPINNAKER_LIB Spinnaker PATHS /opt/spinnaker/lib NO_DEFAULT_PATH REQUIRED)

add_executable(bumblebee_node src/bumblebee_node.cpp)

target_include_directories(bumblebee_node PRIVATE ${SPINNAKER_INCLUDE_DIR})
target_link_libraries(bumblebee_node ${SPINNAKER_LIB} ${OpenCV_LIBRARIES})
ament_target_dependencies(bumblebee_node rclcpp sensor_msgs image_transport)

# Embed the Spinnaker lib path in the installed binary's RPATH so it resolves
# at runtime without needing LD_LIBRARY_PATH.
set_target_properties(bumblebee_node PROPERTIES
  INSTALL_RPATH "/opt/spinnaker/lib"
  BUILD_WITH_INSTALL_RPATH TRUE)

install(TARGETS bumblebee_node DESTINATION lib/${PROJECT_NAME})

ament_package()
