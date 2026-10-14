
cmake_minimum_required(VERSION 3.8)
project(swift_pico)

if(NOT CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -s")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# --- Dependencies -------------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(swift_msgs REQUIRED)
find_package(controller_msg REQUIRED)
find_package(error_msg REQUIRED)
find_package(whycode_interfaces REQUIRED)
find_package(std_msgs REQUIRED)

# Task 2B custom action and service interfaces
find_package(waypoint_navigation REQUIRED)

# --- MuJoCo bridge ------------------------------------------------------------
install(
  PROGRAMS config/mujoco_bridge
  DESTINATION lib/${PROJECT_NAME}
)

install(
  FILES config/libmujoco.so.3.9.0
  DESTINATION lib/${PROJECT_NAME}
)

# --- Launch and configuration -------------------------------------------------
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES config/fastdds_profile.xml
  DESTINATION share/${PROJECT_NAME}/config
)

# --- WhyCode coordinates -------------------------------------------------------
install(
  FILES src/grid_whycode_coordinates.json
  DESTINATION share/${PROJECT_NAME}/scripts
)

# --- C++ PID controller --------------------------------------------------------
add_executable(
  pico_controller_PID
  src/pico_controller_PID.cpp
)

ament_target_dependencies(
  pico_controller_PID
  rclcpp
  swift_msgs
  controller_msg
  error_msg
  whycode_interfaces
  std_msgs
)

install(
  TARGETS pico_controller_PID
  DESTINATION lib/${PROJECT_NAME}
)

# --- Existing Python controllers ----------------------------------------------
install(
  PROGRAMS
    src/pico_controller_PID.py
    src/pico_controller_LQR.py
  DESTINATION lib/${PROJECT_NAME}
)

# --- Task 2B Python nodes -------------------------------------------------------
install(
  PROGRAMS
    src/waypoint_service.py
    src/pico_client.py
    src/pico_server.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

install(PROGRAMS
  src/waypoint_service.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME waypoint_service
)

install(PROGRAMS
  src/pico_client.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pico_client
)

install(PROGRAMS
  src/pico_server.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pico_server
)
