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

# Separate device-free oracle; the existing live planner target is unchanged.
add_executable(experiment_oracle EXCLUDE_FROM_ALL experiment_oracle.cpp)
target_link_libraries(experiment_oracle PRIVATE
  moveit_core::moveit_planning_scene moveit_core::moveit_robot_model
  moveit_core::moveit_robot_state moveit_core::moveit_robot_trajectory
  moveit_core::moveit_trajectory_processing ompl::ompl JsonCpp::JsonCpp)
target_compile_options(experiment_oracle PRIVATE -Wall -Wextra)
