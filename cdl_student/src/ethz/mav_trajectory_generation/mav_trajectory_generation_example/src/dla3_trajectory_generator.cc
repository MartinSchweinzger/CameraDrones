#include <mav_trajectory_generation_example/dla3_trajectory_generator.h>
#include "mav_trajectory_generation/test_utils.h"

DLA3TrajectoryGenerator::DLA3TrajectoryGenerator(ros::NodeHandle& nh, ros::NodeHandle& pnh) :
    nh_(nh),
    pnh_(pnh),
    max_v_(2.0),
    max_a_(4.0) //,
    //current_velocity_(Eigen::Vector3d::Zero()),
    //current_pose_(Eigen::Affine3d::Identity()) 
  {
      
  // Load params
  if (!nh_.getParam(ros::this_node::getName() + "/max_v", max_v_)){
    ROS_WARN("[example_planner] param max_v not found");
    max_v_ = 2.0;
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_a", max_a_)){
    ROS_WARN("[example_planner] param max_a not found");
    max_a_ = 0.5;
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_j", max_j_)){
    ROS_WARN("[example_planner] param max_v not found");
    max_j_ = 1.5;
  }
  if (!nh_.getParam(ros::this_node::getName() + "/max_s", max_s_)){
    ROS_WARN("[example_planner] param max_a not found");
    max_s_ = 3.0;
  }
  if (!nh_.getParam(ros::this_node::getName() + "/use_ros_service", use_ros_service)){
    ROS_WARN("[example_planner] param use_ros_service not found");
    use_ros_service = false;
  } 

  ROS_INFO_STREAM("param max_v_: " << max_v_ );
  ROS_INFO_STREAM("param max_a_: " << max_a_ );
  ROS_INFO_STREAM("param max_j_: " << max_j_ );
  ROS_INFO_STREAM("param max_s_: " << max_s_ );
  ROS_INFO_STREAM("param use_ros_service: " << use_ros_service );

  // create publisher for RVIZ markers
  pub_smooth_trajectory_markers_ =
      pnh.advertise<visualization_msgs::MarkerArray>("smooth_trajectory_markers", 0);

  pub_smooth_trajectory_ =
      pnh.advertise<mav_planning_msgs::PolynomialTrajectory4D>("smooth_trajectory", 0);

  // subscriber for Odometry
  //sub_odom_ =
  //    nh.subscribe("uav_pose", 1, &DLA3TrajectoryGenerator::uavOdomCallback, this);
  planned_trajectory_sub = nh.subscribe<mav_planning_msgs::PolynomialTrajectory4D>(
              "path_planner/planned_trajectory", 1,
              &DLA3TrajectoryGenerator::plannedTrajectoryCallback, this);

  // ROS services
  navi_set_traj4d = nh_.serviceClient<icg_srvs::SetTrajectory4DParameter>("navigation_controller/load_trajectory4d");
  //   TSI
  sub_in_ = pnh.subscribe( "in", 50, &DLA3TrajectoryGenerator::onIn, this);
  pub_out_ = pnh.advertise<std_msgs::Int32>("out",10);
  pub_tsi_out_ = pnh.advertise<std_msgs::Int32>("in",10);
}


void DLA3TrajectoryGenerator::onIn(const std_msgs::Int32::ConstPtr& p_msg) 
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
        // ROS_INFO("TSI: PLANNER_PLAN_TRAJECTORY");
        // do nothing; This is done through the ros-service "load_waypoint".
        } break;    
        default: {
        ROS_INFO("TSI: Unknown task scheduler command.");
        } break;
    }
}

// // Callback to get current Pose of UAV
// void DLA3TrajectoryGenerator::uavOdomCallback(const nav_msgs::Odometry::ConstPtr& odom) {

//   // store current position in our planner
//   tf::poseMsgToEigen(odom->pose.pose, current_pose_);

//   // store current vleocity
//   tf::vectorMsgToEigen(odom->twist.twist.linear, current_velocity_);
// }

// Method to set maximum speed.
void DLA3TrajectoryGenerator::setMaxSpeed(const double max_v) {
  max_v_ = max_v;
}

bool DLA3TrajectoryGenerator::publishTrajectory(const mav_trajectory_generation::Trajectory& trajectory){
  // send trajectory as markers to display them in RVIZ
  visualization_msgs::MarkerArray markers;
  double distance =
      0.2; // Distance by which to seperate additional markers. Set 0.0 to disable.
  std::string frame_id = "world";

  mav_trajectory_generation::drawMavTrajectory(trajectory,
                                               distance,
                                               frame_id,
                                               &markers);
  pub_smooth_trajectory_markers_.publish(markers);

  // send trajectory to be executed on UAV
  mav_planning_msgs::PolynomialTrajectory4D msg;
  mav_trajectory_generation::trajectoryToPolynomialTrajectoryMsg(trajectory,
                                                                 &msg);
  msg.header.frame_id = "world";
  pub_smooth_trajectory_.publish(msg);
  if (false) { // [BUG][FIXED] print-out trajectory, I do this because rostopic echo breaks when receiving a message of this type
    int i = 0;
    for (auto it=msg.segments.begin(); it!=msg.segments.end(); it++) {
      std::cout <<
        "segment["<<i<<"]"<<
        *it << std::endl;
      i++;
    }
  }

  if (use_ros_service) {
    ROS_INFO("Waiting for 'navigation_controller/load_trajectory4d' rosservice to be available!");
    navi_set_traj4d.waitForExistence();
    icg_srvs::SetTrajectory4DParameter srv;
    srv.request.header.frame_id = "world";
    srv.request.header.stamp = ros::Time::now();
    srv.request.traj4d = msg;
    navi_set_traj4d.call(srv);
    ROS_INFO("Trajectory sent to controller through rosservice 'navigation_controller/load_trajectory4d'!");
    pub_tsi_out_.publish( CdlMsgsCommon::createMsg( CdlMsgsCommon::NAVI_FLY_TRAJECTORY ) );
    ROS_INFO("Sent out TSI command 'NAVI_FLY_TRAJECTORY'!");
  }

  return true;
}


void DLA3TrajectoryGenerator::plannedTrajectoryCallback(const mav_planning_msgs::PolynomialTrajectory4D::ConstPtr &p_msg) {
  bool debug = false;
  if (debug) ROS_INFO("Starting... DLA3TrajectoryGenerator::plannedTrajectoryCallback(...)");
  if (use_ros_service) {
    pub_out_.publish( CdlMsgsCommon::createMsg( CdlMsgsCommon::PLANNER_SMOOTHING_TRAJECTORY ) );
  }
  const mav_planning_msgs::PolynomialTrajectory4D &msg = *p_msg;
  size_t N = msg.segments.size();
  
  if (debug) ROS_INFO("Defining initial and goal positions...");
  Eigen::Vector3d position_i, position_g;
  {
    const mav_planning_msgs::PolynomialSegment4D &segment = msg.segments[0];
    position_i[0] = segment.x[0];
    position_i[1] = segment.y[0];
    position_i[2] = segment.z[0];
  }
  {
    const mav_planning_msgs::PolynomialSegment4D &segment = msg.segments[N-1];
    position_g[0] = segment.x[0];
    position_g[1] = segment.y[0];
    position_g[2] = segment.z[0];
  }
  Eigen::Vector3d velocity_i, velocity_g;
  velocity_i <<   0.0,   0.0,   0.0;
  velocity_g <<   0.0,   0.0,   0.0;
  Eigen::Vector3d acceleration_i, acceleration_g;
  acceleration_i <<   0.0,   0.0,   0.0;
  acceleration_g <<   0.0,   0.0,   0.0;
  Eigen::Vector3d jerk_i, jerk_g;
  jerk_i <<   0.0,   0.0,   0.0;
  jerk_g <<   0.0,   0.0,   0.0;
  Eigen::Vector3d snap_i, snap_g;
  snap_i <<   0.0,   0.0,   0.0;
  snap_g <<   0.0,   0.0,   0.0;


  // 3 Dimensional trajectory => through carteisan space, no orientation
  const int dimension = 3;

  // Array for all waypoints and their constrains
  mav_trajectory_generation::Vertex::Vector vertices;

  // Optimze up to 4th order derivative (SNAP)
  const int derivative_to_optimize =
      mav_trajectory_generation::derivative_order::SNAP;

  // we have 2 vertices:
  // Start = current position
  // end = desired position and velocity

  if (debug) ROS_INFO("Configuring start point...");
  { /******* Configure start point *******/
    mav_trajectory_generation::Vertex start(dimension);
    // set start point constraints to current position and set all derivatives to zero
    //start.makeStartOrEnd(current_pose_.translation(),
    //                     derivative_to_optimize);
    start.makeStartOrEnd(position_i,
                         derivative_to_optimize);

    // set start point's velocity to be constrained to current velocity
    start.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, velocity_i);
    start.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acceleration_i);
    start.addConstraint(mav_trajectory_generation::derivative_order::JERK, jerk_i);
    start.addConstraint(mav_trajectory_generation::derivative_order::SNAP, snap_i);

    // add waypoint to list
    vertices.push_back(start);
  }

  if (debug) ROS_INFO("Adding all intermediate waypoints...");
  // *** Adding all intermediate waypoints ***
  // Create the vertices for the points and lines
  for (size_t i=1; i<N-1; i++)
  {
    mav_trajectory_generation::Vertex vertex(dimension);
    Eigen::Vector3d p_eigen;

    const mav_planning_msgs::PolynomialSegment4D &segment = msg.segments[i];
    p_eigen << segment.x[0], segment.y[0], segment.z[0];

    // set vertex point's position to be constrained to planned trajetory waypoint
    vertex.addConstraint(mav_trajectory_generation::derivative_order::POSITION,
                         p_eigen);

    vertices.push_back(vertex);
  } 
  // --- Adding all intermediate waypoints ---

  if (debug) ROS_INFO("Configuring end point...");
  { /******* Configure end point *******/
    mav_trajectory_generation::Vertex end(dimension);
    // set end point constraints to desired position and set all derivatives to zero
    end.makeStartOrEnd(position_g,
                      derivative_to_optimize);

    // set end point's velocity to be constrained to desired velocity
    end.addConstraint(mav_trajectory_generation::derivative_order::VELOCITY, velocity_g);
    end.addConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, acceleration_g);
    end.addConstraint(mav_trajectory_generation::derivative_order::JERK, jerk_g);
    end.addConstraint(mav_trajectory_generation::derivative_order::SNAP, snap_g);

    // add waypoint to list
    vertices.push_back(end);
  }
  // mav_trajectory_generation::NonlinearOptimizationParameters::{kSquaredTime, kRichterTime, ...}
  bool use_alg_1 =  true; // {kSquaredTime, kRichterTime}
  bool use_alg_2 = false; // {kSquaredTimeAndConstraints, kRichterTimeAndConstraints}
  bool use_alg_3 = false; // {kMellingerOuterLoop}

  mav_trajectory_generation::Trajectory trajectory;

  if (use_alg_1) {
    // From first half of: test_polynomial_optimization.cpp : TEST_P(PolynomialOptimizationTests, UnconstrainedNonlinear) {...}
    // estimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.algorithm = nlopt::LN_SBPLX; // nlopt::LN_SBPLX; default (bad results): nlopt::LN_BOBYQA
    parameters.f_rel = 0.00005;
    parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kSquaredTime;
    // parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kRichterTime;

    mav_trajectory_generation::timing::Timer timer_setup("setup_nonlinear_time_only");
    const int Np = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<Np> opt(dimension, parameters);
    // constrain velocity and acceleration
    // set up optimization problem
    opt.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    if (debug) ROS_INFO("Adding constraints for velocity and acceleration...");
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, max_j_);
    opt.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::SNAP, max_s_);
    // opt.solveLinear(); // [DONE] This seems not to be necessary
    double initial_cost = opt.getTotalCostWithSoftConstraints();
    timer_setup.Stop();
    // solve trajectory
    mav_trajectory_generation::timing::Timer timer_solve("solve_nonlinear_time_only");
    if (debug) ROS_INFO("Calculating of trajectory (through optimization)...");
    int ret = opt.optimize();
    timer_solve.Stop();
    if (debug) ROS_INFO("Calculation of trajectory finished (through optimization)...");
    if (ret == nlopt::FAILURE)
    {
      ROS_ERROR("Optimization Error. Failure in NLOPT");
    }
    double final_cost = opt.getTotalCostWithSoftConstraints();
    // get trajectory as polynomial parameters
    opt.getTrajectory(&trajectory);
  }
  if (use_alg_2) {
    // From second half of: test_polynomial_optimization.cpp : TEST_P(PolynomialOptimizationTests, UnconstrainedNonlinear) {...}
    // estimate initial segment times
    std::vector<double> segment_times;
    segment_times = estimateSegmentTimes(vertices, max_v_, max_a_);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.algorithm = nlopt::LN_SBPLX; // nlopt::LN_SBPLX; default (bad results): nlopt::LN_BOBYQA
    parameters.f_rel = 0.00005;
    parameters.time_penalty = 0.1*500.0;
    parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kSquaredTimeAndConstraints;
    // parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kRichterTimeAndConstraints;

    mav_trajectory_generation::timing::Timer timer_setup2("setup_nonlinear_time_and_derivatives");
    const int Np = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<Np> opt2(dimension, parameters);
    // constrain velocity and acceleration
    // set up optimization problem
    opt2.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    if (debug) ROS_INFO("Adding constraints for velocity and acceleration...");
    opt2.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt2.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
    opt2.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, max_j_);
    opt2.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::SNAP, max_s_);
    timer_setup2.Stop();
    // solve trajectory
    mav_trajectory_generation::timing::Timer timer_solve2("solve_nonlinear_time_and_derivatives");
    if (debug) ROS_INFO("Calculating of trajectory (through optimization)...");
    int ret = opt2.optimize();
    timer_solve2.Stop();
    if (debug) ROS_INFO("Calculation of trajectory finished (through optimization)...");
    if (ret == nlopt::FAILURE)
    {
      ROS_ERROR("Optimization Error. Failure in NLOPT");
    }
    double final_cost2 = opt2.getTotalCostWithSoftConstraints();
    // get trajectory as polynomial parameters
    opt2.getTrajectory(&trajectory);
  }
  if (use_alg_3) {
    // From: test_polynomial_optimization.cpp : TEST_P(PolynomialOptimizationTests, TimeScaling) {...}
    // estimate initial segment times
    std::vector<double> segment_times;
    double time_factor = 1.0;
    constexpr double kTolerance = 1e-3;
    segment_times = // Allocate using the ramp.
        estimateSegmentTimesVelocityRamp(vertices, max_v_, max_a_, time_factor);

    // Set up polynomial solver with default params
    mav_trajectory_generation::NonlinearOptimizationParameters parameters;
    parameters.algorithm = nlopt::LD_LBFGS; // default: nlopt::LD_LBFGS ([DONE] test): nlopt::LN_SBPLX - does not work, error message: Check failed: !gradient.empty() only with gradients possible, choose a gradient based method
    parameters.f_rel = 0.00005;
    parameters.print_debug_info_time_allocation = false;
    parameters.time_alloc_method = mav_trajectory_generation::NonlinearOptimizationParameters::kMellingerOuterLoop;

    mav_trajectory_generation::timing::Timer timer_setup2("setup_nonlinear_time_and_derivatives");
    const int Np = 10;
    mav_trajectory_generation::PolynomialOptimizationNonLinear<Np> opt3(dimension, parameters);
    // constrain velocity and acceleration
    // set up optimization problem
    opt3.setupFromVertices(vertices, segment_times, derivative_to_optimize);
    if (debug) ROS_INFO("Adding constraints for velocity and acceleration...");
    opt3.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::VELOCITY, max_v_);
    opt3.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::ACCELERATION, max_a_);
    opt3.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::JERK, max_j_);
    opt3.addMaximumMagnitudeConstraint(mav_trajectory_generation::derivative_order::SNAP, max_s_);
    timer_setup2.Stop();
    // solve trajectory
    mav_trajectory_generation::timing::Timer timer_solve2("solve_nonlinear_time_and_derivatives");
    if (debug) ROS_INFO("Calculating of trajectory (through optimization)...");
    /* // I do not think that these steps are necessary
    opt3.solveLinear();
    opt3.getTrajectory(&trajectory);
    double initial_cost3 = opt3.getCost();
    opt3.scaleSegmentTimesWithViolation();
    opt3.getTrajectory(&trajectory);
    double scaled_cost3 = opt3.getCost();
    */ 
    int ret = opt3.optimize();
    timer_solve2.Stop();
    if (debug) ROS_INFO("Calculation of trajectory finished (through optimization)...");
    if (ret == nlopt::FAILURE)
    {
      ROS_ERROR("Optimization Error. Failure in NLOPT");
    }
    double final_cost3 = opt3.getCost();
    // get trajectory as polynomial parameters
    opt3.getTrajectory(&trajectory);
  }

  if (use_ros_service) {
    pub_out_.publish( CdlMsgsCommon::createMsg( CdlMsgsCommon::PLANNER_TRAJECTORY_SMOOTHED ) );
  }

  // publish smooth trajectory
  publishTrajectory(trajectory);

  // get mav values of derivatives over the generated trajectory: v_max, a_max, j_max, s_max
  double v_max_traj = mav_trajectory_generation::getMaximumMagnitude(trajectory, mav_trajectory_generation::derivative_order::VELOCITY);
  double a_max_traj = mav_trajectory_generation::getMaximumMagnitude(trajectory, mav_trajectory_generation::derivative_order::ACCELERATION);
  double j_max_traj = mav_trajectory_generation::getMaximumMagnitude(trajectory, mav_trajectory_generation::derivative_order::JERK);
  double s_max_traj = mav_trajectory_generation::getMaximumMagnitude(trajectory, mav_trajectory_generation::derivative_order::SNAP);
  double t_traj = trajectory.getMaxTime();
  ROS_INFO_STREAM(" ** generated trajectory ** " << " t_traj: " << t_traj
                  << " v_max: " << v_max_traj << " a_max: " << a_max_traj 
                  << " j_max: " << j_max_traj << " s_max: " << s_max_traj);

  if (debug) ROS_INFO("Ending...   DLA3TrajectoryGenerator::plannedTrajectoryCallback(...)");
  return;
}

