/*
 * Simple example that shows how to sample a trajectories generated using the mav_trajectory_generation routines.
 *
 * Code mostly based on the code:
 *   mav_trajectory_generation_ros/trajectory_sampler_node.h / cpp
 * 
 * Created on: January 2020
 * Author: (P2F) Jesus Pestana Puerta <jesus.pestana@pro2future.at>
 *
 * [TODO]: 
 * 
 */

#include <mav_trajectory_generation_example/dla3_trajectory_sampler.h>

DLA3TrajectorySampler::DLA3TrajectorySampler(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
  nh_(nh), pnh_(pnh),
  dt_(0.01),
  current_sample_time_(0.0) {
  pnh_.param("dt", dt_, dt_);
  std::string output_topic;
  pnh.param( std::string("output_topic"), output_topic, std::string("full_odometry"));

  smooth_trajectory4d_sub = pnh_.subscribe<mav_planning_msgs::PolynomialTrajectory4D>(
    "smooth_trajectory4d", 10,
    &DLA3TrajectorySampler::smoothTrajectory4DCallback, this);
  
  const bool oneshot = false;
  const bool autostart = false;
  publish_timer_ = nh_.createTimer(ros::Duration(dt_),
                                   &DLA3TrajectorySampler::commandTimerCallback,
                                   this, oneshot, autostart);
  command_pub_ = pnh_.advertise<trajectory_msgs::MultiDOFJointTrajectory>("command/trajectory", 1);
  pubFullOdometry_ = pnh_.advertise<icg_msgs::FullOdometryWOCov>(output_topic, 10);
}

void DLA3TrajectorySampler::smoothTrajectory4DCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr& p_msg) {
  ROS_INFO("BEGIN: smoothTrajectory4DCallback(...)");
  const mav_planning_msgs::PolynomialTrajectory4D &msg = *p_msg;

  if (msg.segments.empty()) {
    ROS_WARN("Trajectory sampler: received empty waypoint message");
    return;
  } else {
    ROS_INFO("Trajectory sampler: received %lu waypoints", msg.segments.size());
  }

  ROS_INFO("Received trajectory print-out...");
  std::cout << "header:" << msg.header << std::endl;
  { // print-out trajectory, I do this because rostopic echo breaks when receiving a message of this type
    int i = 0;
    for (auto it=msg.segments.begin(); it!=msg.segments.end(); it++) {
      std::cout <<
        "segment["<<i<<"]"<<
        *it << std::endl;
      i++;
    }
  }

  bool success = mav_trajectory_generation::polynomialTrajectoryMsgToTrajectory( msg, &trajectory_);
  if (!success) {
    return;
  }
  processTrajectory();

  ROS_INFO("END:   smoothTrajectory4DCallback(...)");
}

void DLA3TrajectorySampler::processTrajectory() {
  publish_timer_.stop();
  publish_timer_.start();
  current_sample_time_ = 0.0;
  start_time_ = ros::Time::now();
}

void DLA3TrajectorySampler::commandTimerCallback(const ros::TimerEvent&) {
  if (current_sample_time_ <= trajectory_.getMaxTime()) {
    trajectory_msgs::MultiDOFJointTrajectory msg;
    mav_msgs::EigenTrajectoryPoint trajectory_point;
    bool success = mav_trajectory_generation::sampleTrajectoryAtTime(
        trajectory_, current_sample_time_, &trajectory_point);
    if (!success) {
      publish_timer_.stop();
    }
    mav_msgs::msgMultiDofJointTrajectoryFromEigen(trajectory_point, &msg);
    msg.points[0].time_from_start = ros::Duration(current_sample_time_);
    ros::Time current_ideal_time_ = start_time_ + ros::Duration(current_sample_time_);
    msg.header.frame_id = "world";
    std::string child_frame_id("state_ref");
    msg.header.stamp = current_ideal_time_;
    command_pub_.publish(msg);
    current_sample_time_ += dt_;

    {
      tf::Transform transform;
      const geometry_msgs::Vector3 &t = msg.points[0].transforms[0].translation;
      const geometry_msgs::Quaternion &q = msg.points[0].transforms[0].rotation;
      transform.setOrigin(tf::Vector3( t.x, t.y, t.z));
      transform.setRotation(tf::Quaternion( q.x, q.y, q.z, q.w));
      tf_broadcaster_.sendTransform(
          tf::StampedTransform(
              transform, current_ideal_time_,
              msg.header.frame_id, child_frame_id));
    }


    icg_msgs::FullOdometryWOCov::Ptr p_msgFullOdometryWOCov( new icg_msgs::FullOdometryWOCov() );
    {
      icg_msgs::FullOdometryWOCov &msg_o = *p_msgFullOdometryWOCov;
      msg_o.header.stamp = current_ideal_time_;
      msg_o.header.frame_id = msg.header.frame_id;
      msg_o.child_frame_id = child_frame_id;
      const geometry_msgs::Vector3 &t = msg.points[0].transforms[0].translation;
      msg_o.pose.position.x = t.x;
      msg_o.pose.position.y = t.y;
      msg_o.pose.position.z = t.z;
      const geometry_msgs::Quaternion &q = msg.points[0].transforms[0].rotation;
      msg_o.pose.orientation.x = q.x;
      msg_o.pose.orientation.y = q.y;
      msg_o.pose.orientation.z = q.z;
      msg_o.pose.orientation.w = q.w;
      const geometry_msgs::Vector3 &v = msg.points[0].velocities[0].linear;
      msg_o.twist.linear.x = v.x;
      msg_o.twist.linear.y = v.y;
      msg_o.twist.linear.z = v.z;
      const geometry_msgs::Vector3 &w = msg.points[0].velocities[0].angular;
      msg_o.twist.angular.x = w.x; 
      msg_o.twist.angular.y = w.y;
      msg_o.twist.angular.z = w.z;
      const geometry_msgs::Vector3 &a = msg.points[0].accelerations[0].linear;
      msg_o.accel.linear.x = a.x;
      msg_o.accel.linear.y = a.y;
      msg_o.accel.linear.z = a.z;
      const geometry_msgs::Vector3 &dwdt = msg.points[0].accelerations[0].angular;
      msg_o.accel.angular.x = dwdt.x; 
      msg_o.accel.angular.y = dwdt.y;
      msg_o.accel.angular.z = dwdt.z;
    }
    pubFullOdometry_.publish(p_msgFullOdometryWOCov);

  } else {
    publish_timer_.stop();
  }
}


