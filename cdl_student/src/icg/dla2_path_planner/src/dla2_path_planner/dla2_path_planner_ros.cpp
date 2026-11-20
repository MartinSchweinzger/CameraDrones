/*
 * DLA2 Path Planner ROS - dla2_path_planner_ros.cpp
 *
 *  Author: Jesus Pestana <pestana@icg.tugraz.at>
 *  Created on: Dec 19, 2019
 *  Modified: Jun Zhang <jun.zhang@tugraz.at> (October 2024)
 *
 */

#include <dla2_path_planner/dla2_path_planner_ros.h>

DLA2PathPlanner::DLA2PathPlanner(ros::NodeHandle &n, ros::NodeHandle &pn, int argc, char** argv) :
    pnode_(pn),
    node_(n),
    traj_planning_successful(false),
    traj_planning_with_simplification_successful(false)
{   
    // ROS topics
    current_position_sub = pnode_.subscribe("current_position", 10, &DLA2PathPlanner::currentPositionCallback, this);
    goal_position_sub = pnode_.subscribe("goal_position", 10, &DLA2PathPlanner::goalPositionCallback, this);
    trajectory_pub = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory_raw", 1);
    trajectory_simplified_pub = pnode_.advertise<mav_planning_msgs::PolynomialTrajectory4D>("planned_trajectory", 1);
    tfs_sub = pnode_.subscribe("current_tf", 20, &DLA2PathPlanner::currentPositionTFSCallback, this);
    service_load_waypoint_param = pnode_.advertiseService("load_waypoint", &DLA2PathPlanner::serviceServerLoadWaypoint, this);
    
    //   TSI
    sub_in_ = pnode_.subscribe( "in", 50, &DLA2PathPlanner::onIn, this);
    pub_out_ = pnode_.advertise<std_msgs::Int32>("out",10);

    current_position.x = 1.50; current_position.y = -1.50; current_position.z = 1.10;
    goal_position.x = 1.; goal_position.y = 1.; goal_position.z = 1.;

    // Parse the arguments, returns true if successful, false otherwise
    std::string bt_file_rel_path;
    if (argParse(argc, argv, &runTime, &plannerType, &objectiveType, &outputFile, &bt_file_rel_path))
    {
        // Return with success
        ROS_INFO("DLA2PathPlanner::DLA2PathPlanner(...) argParse success!");
    } else {
        // Return with error - Modified argParse to make this equivalent to giving no arguments.
        ROS_INFO("DLA2PathPlanner::DLA2PathPlanner(...) argParse error!");
    }

    // initialize distance map
    std::string dla2_path_planner_pkg_path = ros::package::getPath("dla2_path_planner");
    ROS_INFO_STREAM("dla2_path_planner_pkg_path: " << dla2_path_planner_pkg_path);
    std::string dla2_path_planner_pkg_maps_folder_path = dla2_path_planner_pkg_path + "/maps";
    ROS_INFO_STREAM("dla2_path_planner_pkg_maps_folder_path: " << dla2_path_planner_pkg_maps_folder_path);
    bt_file_path = dla2_path_planner_pkg_maps_folder_path + "/" + bt_file_rel_path;
    ROS_INFO_STREAM("bt_file_path: " << bt_file_path);
    p_distmap = std::shared_ptr<DLA2DistanceMap>( new DLA2DistanceMap( bt_file_path ) );
    pub_out_.publish( CdlMsgsCommon::createMsg( CdlMsgsCommon::PLANNER_IDLE ) );
}

DLA2PathPlanner::~DLA2PathPlanner() {

}


void DLA2PathPlanner::onIn(const std_msgs::Int32::ConstPtr& p_msg) 
{
    // filter task_scheduler commands to the range of this node
    if ( !( ( 200 <= p_msg->data ) && ( p_msg->data < 250 ) ) )
        return;
    switch (p_msg->data) {
    case CdlMsgsCommon::PLANNER_SET_IDLE: {
        ROS_INFO("TSI: PLANNER_SET_IDLE");
        // do nothing;
        } break;
    case CdlMsgsCommon::PLANNER_PLAN_TRAJECTORY: {
        ROS_INFO("TSI: PLANNER_PLAN_TRAJECTORY");
        // do nothing; This is done through the ros-service "load_waypoint".
        } break;    
        default: {
        ROS_INFO("TSI: Unknown task scheduler command.");
        } break;
    }
}

void DLA2PathPlanner::currentPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg) {
    current_position = *p_msg;
    ROS_INFO_STREAM("New current position, x: " << current_position.x << "; y: " << current_position.y << "; z: " << current_position.z);
}
      
void DLA2PathPlanner::currentPositionTFSCallback(const geometry_msgs::TransformStamped::ConstPtr& p_msg) {
    //if (DEBUG_OUTPUT) ROS_INFO("DLA2PathPlanner::currentPositionTFSCallback(...) Starting...");
    current_position.x = p_msg->transform.translation.x;
    current_position.y = p_msg->transform.translation.y;
    current_position.z = p_msg->transform.translation.z;
    //ROS_INFO_STREAM("New current position, x: " << current_position.x << "; y: " << current_position.y << "; z: " << current_position.z);
    //if (DEBUG_OUTPUT) ROS_INFO("DLA2PathPlanner::currentPositionTFSCallback(...) Ending...");
}

void DLA2PathPlanner::goalPositionCallback(const geometry_msgs::Point::ConstPtr& p_msg) {
    goalPositionCallbackHelper(*p_msg);
}

bool DLA2PathPlanner::goalPositionCallbackHelper(const geometry_msgs::Point& msg) {
    goal_position = msg;
    ROS_INFO_STREAM("New goal position, x: " << goal_position.x << "; y: " << goal_position.y << "; z: " << goal_position.z);

    plan();

    if (traj_planning_successful) {
        convertOMPLPathToMsg(p_last_traj_ompl, last_traj_msg);
        mav_planning_msgs::PolynomialTrajectory4D::Ptr p_traj_msg = 
            mav_planning_msgs::PolynomialTrajectory4D::Ptr( new mav_planning_msgs::PolynomialTrajectory4D( last_traj_msg ) );
        trajectory_pub.publish(p_traj_msg);
    } else {
        return false;
    }
    if (traj_planning_with_simplification_successful) {
        convertOMPLPathToMsg(p_last_traj_ompl_simplified, last_traj_simplified_msg);
        mav_planning_msgs::PolynomialTrajectory4D::Ptr p_traj_simplified_msg = 
            mav_planning_msgs::PolynomialTrajectory4D::Ptr( new mav_planning_msgs::PolynomialTrajectory4D( last_traj_simplified_msg ) );
        trajectory_simplified_pub.publish(p_traj_simplified_msg);
    } else {
        return false;
    }

    return true;
}

bool DLA2PathPlanner::serviceServerLoadWaypoint(icg_srvs::SetPlannerWaypointParameter::Request  &req, 
                                                icg_srvs::SetPlannerWaypointParameter::Response &res ) {
    const geometry_msgs::Point& gw = req.goal_position; // gw: goal waypoint
    // plan trajectory
    pub_out_.publish( CdlMsgsCommon::createMsg( CdlMsgsCommon::PLANNER_PLANNING_TRAJECTORY ) );
    bool success = goalPositionCallbackHelper(gw);
    ros::Duration(1.0).sleep(); 
    pub_out_.publish( CdlMsgsCommon::createMsg( CdlMsgsCommon::PLANNER_TRAJECTORY_PLANNED ) );
    // send response to ros-service client
    std::stringstream strs;
    if (success) {
        strs << "[" << ros::Time::now() << "][path_planner] Trajectory to goal waypoint: ["<<gw.x<<", "<<gw.y<<", "<<gw.z<<"] planned succesfully!"; 
    } else {
        strs << "[" << ros::Time::now() << "][path_planner] Planning of trajectory to goal waypoint: ["<<gw.x<<", "<<gw.y<<", "<<gw.z<<"] failed!"; 
    }
    res.message = strs.str();
    res.success = success;
    return success;                   
}

void DLA2PathPlanner::convertOMPLPathToMsg( std::shared_ptr<ompl::geometric::PathGeometric> &path, mav_planning_msgs::PolynomialTrajectory4D &msg) {
    msg.segments.clear();

    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "world"; // "odom"

    std::vector<ompl::base::State *> &states = path->getStates();
    size_t N = states.size();
    for (size_t i = 0; i<N; i++) {
        const ompl::base::State *p_s = states[i];
        const double &x_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[0];
        const double &y_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[1];
        const double &z_s = p_s->as<ob::RealVectorStateSpace::StateType>()->values[2];
        double yaw_s = 0.;
        ROS_INFO_STREAM("states["<< i <<"], x_s: " << x_s << "; y_s: " << y_s << "; z_s: " << z_s);

        mav_planning_msgs::PolynomialSegment4D segment;
        segment.header = msg.header;
        segment.num_coeffs = 0;
        segment.segment_time = ros::Duration(0.);
        
        segment.x.push_back(x_s);
        segment.y.push_back(y_s);
        segment.z.push_back(z_s);
        segment.yaw.push_back(yaw_s);
        msg.segments.push_back(segment);
    }
}

void DLA2PathPlanner::plan()
{
    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    auto space(std::make_shared<ob::RealVectorStateSpace>(3));

    // Set the bounds of space to be in [0,1].
    const octomap::point3d &min = p_distmap->getMapMin();
    const octomap::point3d &max = p_distmap->getMapMax();
    {
        ompl::base::RealVectorBounds map_bounds(3);
        map_bounds.setLow( 0, min.x());
        map_bounds.setHigh( 0, max.x());
        map_bounds.setLow( 1, min.y());
        map_bounds.setHigh( 1, max.y());
        map_bounds.setLow( 2, min.z());
        map_bounds.setHigh( 2, max.z());
        space->setBounds( map_bounds );
    }

    // Construct a space information instance for this state space
    auto si(std::make_shared<ob::SpaceInformation>(space));

    // ------------------------------------[STUDENT TODO]----------------------------------------------- //

    // create a new validatychecker class in the helper_function.h, 
    // to check the occupancy and clearance with the dynamicEDT3D library. Example reference: 
    // https://github.com/OctoMap/octomap/blob/devel/dynamicEDT3D/src/examples/exampleEDTOctomap.cpp
    // After creating the class, change the "basic_collision_checker" to true and uncomment the line in
    // the else condition, then recompile and run to check the new planned path. 

    bool basic_collision_checker = true; 
    // Set the object used to check which states in the space are valid
    if (basic_collision_checker){ // using simple collision checker 
        si->setStateValidityChecker(std::make_shared<ValidityChecker3D>(si));
    }
    else { // using collision checker from dynamic_edt_3d
        // si->setStateValidityChecker(std::make_shared<ValidityChecker3DWDistMap>(si,p_distmap));
    }

    // ------------------------------------------------------------------------------------------------- //
        
    //ROS_INFO_STREAM("p_distmap->getResolution(): " << p_distmap->getResolution());
    //ROS_INFO_STREAM("(max-min).norm(): " << (max-min).norm());
    if (basic_collision_checker) { // setStateValidityCheckingResolution
        const octomap::point3d &min = p_distmap->getMapMin();
        const octomap::point3d &max = p_distmap->getMapMax();
        si->setStateValidityCheckingResolution( p_distmap->getResolution() / (max-min).norm() / 2.0 ); // 0.03 = 3%
    }
    else { // MotionValidator
        si->setMotionValidator(std::make_shared<MotionValidator3DWDistMap>(si,p_distmap));
    }
    si->setup();

    // Before: Set our robot's starting state to be the bottom-left corner of
    //           the environment, or (0,0,0) 
    // Now: To the current_position.
    ob::ScopedState<> start(space);
    start->as<ob::RealVectorStateSpace::StateType>()->values[0] = current_position.x;
    start->as<ob::RealVectorStateSpace::StateType>()->values[1] = current_position.y;
    start->as<ob::RealVectorStateSpace::StateType>()->values[2] = current_position.z;

    // Before: Set our robot's goal state to be the top-right corner of the
    //           environment, or (1,1).
    // Now: To the goal_position.
    ob::ScopedState<> goal(space);
    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = goal_position.x;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = goal_position.y;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = goal_position.z;

    // Create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal);

    // Create the optimization objective specified by our command-line argument.
    // This helper function is simply a switch statement.
    pdef->setOptimizationObjective(allocateObjective(si, objectiveType));

    // Construct the optimal planner specified by our command line argument.
    // This helper function is simply a switch statement.
    ob::PlannerPtr optimizingPlanner = allocatePlanner(si, plannerType);

    // Set the problem instance for our planner to solve
    optimizingPlanner->setProblemDefinition(pdef);
    optimizingPlanner->setup();

    // attempt to solve the planning problem in the given runtime
    ob::PlannerStatus solved = optimizingPlanner->solve(runTime);
    std::cout << "Planning step done. PlannerStatus:" << solved.asString() << std::endl;

    if ( ob::PlannerStatus::StatusType(solved) == ob::PlannerStatus::StatusType::EXACT_SOLUTION )
    {
        // Output the length of the path found
        std::cout
            << optimizingPlanner->getName()
            << " found a solution of length "
            << pdef->getSolutionPath()->length()
            << " with an optimization objective value of "
            << pdef->getSolutionPath()->cost(pdef->getOptimizationObjective()) << std::endl;

        // If a filename was specified, output the path as a matrix to
        // that file for visualization
        if (!outputFile.empty())
        {
            std::ofstream outFile(outputFile.c_str());
            std::static_pointer_cast<og::PathGeometric>(pdef->getSolutionPath())->
                printAsMatrix(outFile);
            outFile.close();
        }

        p_last_traj_ompl = std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());
        traj_planning_successful = true;

        // [INFO FOR STUDENT] From here onwards, this is the part of trajectory simplifier and smoother
        // SimplifyPath - code similar to ompl example ompl/tests/geometric/2d/2dpath_simplifying.cpp
        bool do_run_path_simplifier = true;
        if (do_run_path_simplifier) {
            int runs = 20;

            std::shared_ptr<ompl::geometric::PathGeometric> p_path_before_ =
               std::static_pointer_cast<ompl::geometric::PathGeometric>(pdef->getSolutionPath());

            ompl::base::GoalPtr p_goal_ = pdef->getGoal();
            ompl::base::OptimizationObjectivePtr p_obj_ = pdef->getOptimizationObjective();

            ompl::geometric::PathSimplifier simplifier( si, p_goal_, p_obj_);

            double cost_after = 0.0;
            ompl::base::Cost original_cost = p_path_before_->cost(p_obj_);
            std::shared_ptr<ompl::geometric::PathGeometric> p_path_after_ =
               std::shared_ptr<ompl::geometric::PathGeometric>(new ompl::geometric::PathGeometric(*p_path_before_));
            //ompl::geometric::PathGeometric *path;
            double l_path_before = p_path_before_->length();
            for (int i = 0; i < runs; i++) {
                double snapToVertex = 2./l_path_before;
                snapToVertex = snapToVertex > 0.1 ? 0.1 : snapToVertex;
                // double snapToVertex = 0.1; // 0.1 (less points dronespace) // 0.05 (less points) // 0.025 (quite good) // 0.01 (more points) // original value: 0.005
                //snapToVertex = 0.15; //0.005 + double(i)/double(runs)*0.200; // test for mav_trajectory_generation::NonlinearOptimizationParameters::kSquaredTimeAndConstraints;
                // I am not sure that perturbPath is cost-aware anymore...
                //simplifier.perturbPath(*p_path_after_, 2.0, 100, 100, snapToVertex); // test for mav_trajectory_generation::NonlinearOptimizationParameters::kSquaredTimeAndConstraints;
                simplifier.shortcutPath(*p_path_after_, 100, 100, 0.33, snapToVertex);
                // reduceVertices is not cost-aware, I increased value of snapToVertex instead
                //simplifier.reduceVertices(*p_path_after_, 100, 100, 0.33);
                
                cost_after = p_path_after_->cost(p_obj_).value();
                ROS_INFO("perturbPath+shortcutPath; cost_after: %f, original cost: %f", cost_after, original_cost.value());
            }
            unsigned int num_bspline_passess = 0; // 1-2 BSpline pass to make trajectory smoother
            if (num_bspline_passess>0) {
                simplifier.smoothBSpline(*p_path_after_, num_bspline_passess);  
            }
            cost_after = p_path_after_->cost(p_obj_).value();
            ROS_INFO("smoothBSpline; cost_after: %f, original cost: %f", cost_after, original_cost.value());
            
            ROS_INFO_STREAM("p_obj_->isCostBetterThan(cost_after, original_cost): " <<
                             p_obj_->isCostBetterThan(ompl::base::Cost(cost_after), original_cost));
            ROS_INFO_STREAM("p_obj_->isCostEquivalentTo(cost_after, original_cost): " <<
                             p_obj_->isCostEquivalentTo(ompl::base::Cost(cost_after), original_cost));

            p_last_traj_ompl_simplified = p_path_after_;
            traj_planning_with_simplification_successful = true;
        }
    } else {
        std::cout << "No solution found. PlannerStatus:" << solved.asString() << std::endl;
        traj_planning_successful = false;
        traj_planning_with_simplification_successful = false;
    }
}
