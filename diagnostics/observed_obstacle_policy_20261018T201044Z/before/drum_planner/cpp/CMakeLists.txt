cmake_minimum_required(VERSION 3.16)
project(x5_drum_planner LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
find_package(moveit_core REQUIRED)
find_package(ompl REQUIRED)
find_package(jsoncpp REQUIRED)
add_executable(moveit_plan moveit_plan.cpp)
target_link_libraries(moveit_plan PRIVATE
  moveit_core::moveit_planning_scene moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state moveit_core::moveit_robot_trajectory
  moveit_core::moveit_trajectory_processing ompl::ompl JsonCpp::JsonCpp)
target_compile_options(moveit_plan PRIVATE -Wall -Wextra)
