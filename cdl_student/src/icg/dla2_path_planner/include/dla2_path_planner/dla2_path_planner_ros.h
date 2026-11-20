/*
 * DLA2 Path Planner ROS - dla2_path_planner_ros.h
 *
 *  Author: Jesus Pestana <pestana@icg.tugraz.at>
 *  Created on: Dec 19, 2019
 *  Modified: Jun Zhang <jun.zhang@tugraz.at> (October 2024)
 *
 */

#ifndef DLA2_PATH_PLANNER_H_
#define DLA2_PATH_PLANNER_H_

#include "ros/ros.h"
#include <ros/package.h>    // ros::package::getPath(...) function
#include <dla2_path_planner/helper_functions.h>
#include <dla2_path_planner/distance_map.h>
#include <mav_planning_msgs/PolynomialTrajectory4D.h>
// 2D Version
//#include <mav_planning_msgs/Point2D.h>
// 3D Version
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <icg_srvs/SetPlannerWaypointParameter.h>
#include <icg_srvs/SetAprilTagParameter.h>
#include <cdl_common/common.h>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
// For ompl::msg::setLogLevel
#include "ompl/util/Console.h"

// The supported optimal planners, in alphabetical order
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>

// For boost program options
#include <boost/program_options.hpp>
// For string comparison (boost::iequals)
#include <boost/algorithm/string.hpp>
// For std::make_shared
#include <memory>

#include <fstream>

class DLA2PathPlanner
{
public:
    DLA2PathPlanner(ros::NodeHandle &n, ros::NodeHandle &pn, int argc, char** argv);
    ~DLA2PathPlanner();

private:
    ros::NodeHandle &pnode_;
    ros::NodeHandle &node_;

    // parsed arguments
    double runTime;
    optimalPlanner plannerType;
    planningObjective objectiveType;
    std::string outputFile;
    std::string bt_file_path;

    // distance_map - loaded from bt_file
    std::shared_ptr<DLA2DistanceMap> p_distmap;

    void plan();

    // ROS Topics
    ros::Subscriber current_position_sub;
    ros::Subscriber goal_position_sub;
    ros::Subscriber tfs_sub;
    ros::Publisher trajectory_pub, trajectory_simplified_pub;
    ros::Subscriber sub_in_;
    void onIn(const std_msgs::Int32::ConstPtr& p_msg);
    ros::Publisher pub_out_;
    void currentPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg);
    bool goalPositionCallbackHelper(const geometry_msgs::Point& msg);
    void currentPositionTFSCallback(const geometry_msgs::TransformStamped::ConstPtr& p_msg);
    void goalPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg);
    ros::ServiceServer service_load_waypoint_param;
    bool serviceServerLoadWaypoint(icg_srvs::SetPlannerWaypointParameter::Request  &req, 
                                   icg_srvs::SetPlannerWaypointParameter::Response &res );
    void convertOMPLPathToMsg( std::shared_ptr<ompl::geometric::PathGeometric> &path, mav_planning_msgs::PolynomialTrajectory4D &msg );
    geometry_msgs::Point current_position, goal_position;
    bool traj_planning_successful, traj_planning_with_simplification_successful;
    std::shared_ptr<ompl::geometric::PathGeometric> p_last_traj_ompl, p_last_traj_ompl_simplified;
    mav_planning_msgs::PolynomialTrajectory4D last_traj_msg, last_traj_simplified_msg;
};

#endif // DLA2_PATH_PLANNER_H_
