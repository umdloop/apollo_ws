cmake_minimum_required(VERSION 3.5)
project(gps)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(MAVSDK REQUIRED)
add_executable(gps src/MinimalPublisher.cpp)

ament_target_dependencies(gps
        rclcpp
        std_msgs
        sensor_msgs
)
target_link_libraries(gps
        MAVSDK::mavsdk
)
install(TARGETS
        gps
        DESTINATION lib/${PROJECT_NAME})

ament_package()