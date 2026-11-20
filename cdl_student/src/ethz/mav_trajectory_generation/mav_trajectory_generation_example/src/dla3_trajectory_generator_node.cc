/*
 * Simple example that shows a trajectory planner using
 *  mav_trajectory_generation.
 *
 *
 * Launch via
 *   roslaunch mav_trajectory_generation_example example.launch
 *
 * Wait for console to run through all gazebo/rviz messages and then
 * you should see the example below
 *  - After Enter, it receives the current uav position
 *  - After second enter, publishes trajectory information
 *  - After third enter, executes trajectory (sends it to the sampler)
 */

#include  "ros/ros.h"
#include <mav_trajectory_generation_example/dla3_trajectory_generator.h>

#include <iostream>

int main(int argc, char** argv) {
    ROS_INFO("Checking input arguments!");
    ROS_INFO_STREAM("argc: " << argc);
    for (int i=0; i<argc; i++) {
        ROS_INFO_STREAM("argv[" << i << "]: " << argv[i]);
    }
    ROS_INFO("\n");

  /* initialize ros */
  ros::init(argc, argv, "trajectory_generator");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ROS_INFO("Starting DLA3TrajectoryGenerator...");
  DLA3TrajectoryGenerator trajectory_generator(nh, pnh);
  ROS_INFO("DLA3TrajectoryGenerator started...");
  
  ros::spin();

  return 0;
}