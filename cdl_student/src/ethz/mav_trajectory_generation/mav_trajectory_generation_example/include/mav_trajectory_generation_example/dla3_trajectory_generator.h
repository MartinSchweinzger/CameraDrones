#ifndef MAV_TRAJECTORY_GENERATION_DLA3_TRAJECTORY_GENERATOR_H
#define MAV_TRAJECTORY_GENERATION_DLA3_TRAJECTORY_GENERATOR_H

#include <iostream>
#include <ros/ros.h>
#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <eigen_conversions/eigen_msg.h>
#include <mav_trajectory_generation/polynomial_optimization_linear.h>
#include <mav_trajectory_generation/polynomial_optimization_nonlinear.h>
#include <mav_trajectory_generation/timing.h>
#include <mav_trajectory_generation_ros/ros_visualization.h>
#include <mav_trajectory_generation_ros/ros_conversions.h>
#include <icg_srvs/SetTrajectory4DParameter.h>
#include <cdl_common/common.h>

class DLA3TrajectoryGenerator {
 public:
  DLA3TrajectoryGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh);

  void uavOdomCallback(const nav_msgs::Odometry::ConstPtr& pose);

  void setMaxSpeed(double max_v);

  // Plans a trajectory to take off from the current position and
  // fly to the given altitude (while maintaining x,y, and yaw).
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool planTrajectory(const Eigen::VectorXd& goal_pos,
                      const Eigen::VectorXd& goal_vel,
                      const Eigen::VectorXd& start_pos,
                      const Eigen::VectorXd& start_vel,
                      double v_max, double a_max,
                      mav_trajectory_generation::Trajectory* trajectory);
                      
  bool publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory);

 private:
  ros::Publisher pub_smooth_trajectory_markers_;
  ros::Publisher pub_smooth_trajectory_;
  bool use_ros_service; 
  ros::ServiceClient navi_set_traj4d;
  //ros::Subscriber sub_odom_;
  ros::Subscriber sub_in_;
  void onIn(const std_msgs::Int32::ConstPtr& p_msg);
  ros::Publisher pub_out_;
  ros::Publisher pub_tsi_out_;

  ros::Subscriber planned_trajectory_sub;
  void plannedTrajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr &p_msg);

  ros::NodeHandle& nh_, pnh_;
  //Eigen::Affine3d current_pose_;
  ////Eigen::Vector3d current_velocity_;
  ////Eigen::Vector3d current_angular_velocity_;
  double max_v_; // m/s
  double max_a_; // m/s^2
  double max_j_; // m/s^3
  double max_s_; // m/s^4
};

#endif // MAV_TRAJECTORY_GENERATION_DLA3_TRAJECTORY_GENERATOR_H
