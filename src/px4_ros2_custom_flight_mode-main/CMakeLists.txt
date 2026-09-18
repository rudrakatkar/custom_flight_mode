cmake_minimum_required(VERSION 3.5)
project(draw_flight_mode)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wno-unused-parameter)
endif()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(px4_msgs REQUIRED)

# Headers: mode.hpp (D1), mode_curve.hpp (D3), mode_diagonal.hpp (D2), repulsion.hpp (CBF-QP)
include_directories(include ${Eigen3_INCLUDE_DIRS})

add_executable(draw_r src/main.cpp)

ament_target_dependencies(draw_r
    Eigen3
    rclcpp
    std_msgs
    px4_ros2_cpp
    px4_msgs)

install(TARGETS draw_r
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
