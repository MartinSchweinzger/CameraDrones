cmake_minimum_required(VERSION 3.0.2)
project(icg_srvs)

find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs mav_planning_msgs icg_msgs)

add_service_files(DIRECTORY srv
  FILES
  GVMSetParameter.srv SetFloat64Parameter.srv SetPlannerWaypointParameter.srv SetWaypointParameter.srv SetTrajectoryParameter.srv SetTrajectory4DParameter.srv SetAprilTagParameter.srv)

generate_messages(DEPENDENCIES geometry_msgs mav_planning_msgs icg_msgs)

catkin_package(CATKIN_DEPENDS message_runtime geometry_msgs mav_planning_msgs icg_msgs)
