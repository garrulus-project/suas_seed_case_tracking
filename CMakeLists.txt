cmake_minimum_required(VERSION 3.27)
project(suas_seed_case_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(pylon_ros2_camera_interfaces REQUIRED)
find_package(std_srvs REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(image_transport REQUIRED)
find_package(image_geometry REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS imgcodecs)
find_package(Pylon REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nlohmann_json REQUIRED)

set(DEPENDENCIES
    rclcpp
    sensor_msgs
    std_msgs
    pylon_ros2_camera_interfaces
    camera_info_manager
    image_transport
    image_geometry
    cv_bridge
    mavros_msgs
)

include_directories(include)

# ros-independent basler_ace wrapper
add_library(basler_ace
    src/basler_ace.cpp
    src/basler_ace_ros2_wrapper.cpp
)

target_include_directories(basler_ace
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(basler_ace
    pylon::pylon
    ${OpenCV_LIBS}
)

ament_target_dependencies(basler_ace
    rclcpp
    sensor_msgs
    cv_bridge
    pylon_ros2_camera_interfaces
    std_srvs
)

# main seed case tracking node
add_executable(seed_case_tracking_node
    src/seed_case_tracking_node.cpp
    src/seed_case_tracker.cpp
)

target_link_libraries(seed_case_tracking_node
    basler_ace
    ${OpenCV_LIBS}
    nlohmann_json::nlohmann_json
)

ament_target_dependencies(seed_case_tracking_node
    rclcpp
    sensor_msgs
    std_msgs
    cv_bridge
    mavros_msgs
)

# --- Install ---
install(TARGETS basler_ace seed_case_tracking_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION include
)

install(DIRECTORY config launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(basler_ace)

ament_package()
