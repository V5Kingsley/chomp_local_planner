#include <chomp_local_planner/chomp_planner_ros.h>

#include <ros/console.h>

#include <pluginlib/class_list_macros.h>

#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>

#include <nav_core/parameter_magic.h>

PLUGINLIB_EXPORT_CLASS(chomp_local_planner::CHOMPPlannerROS, nav_core::BaseLocalPlanner)

namespace chomp_local_planner
{
CHOMPPlannerROS::CHOMPPlannerROS() : initialized_(false), setup_(false), odom_helper_("odom")
{
  ROS_INFO_NAMED("chomp_planner", "CHOMPPlannerROS created.");
  vel_pub_ = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
}

void CHOMPPlannerROS::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
{
  if (!isInitialized())
  {
    ros::NodeHandle private_nh("~" + name);
    g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
    l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
    tf_ = tf;
    costmap_ros_ = costmap_ros;
    costmap_ros_->getRobotPose(current_pose_);

    costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();

    planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

    /*
        base_local_planner::LocalPlannerLimits limits;
        ros::NodeHandle config_nh("/move_base_node/TrajectoryPlannerROS");
        config_nh.getParam("max_vel_trans", limits.max_vel_trans);
        config_nh.getParam("min_vel_trans", limits.min_vel_trans);
        config_nh.getParam("max_vel_x", limits.max_vel_x);
        config_nh.getParam("min_vel_x", limits.min_vel_x);
        config_nh.getParam("max_vel_y", limits.max_vel_y);
        config_nh.getParam("min_vel_y", limits.min_vel_y);
        config_nh.getParam("max_vel_theta", limits.max_vel_theta);
        config_nh.getParam("min_vel_theta", limits.min_vel_theta);
        config_nh.getParam("acc_lim_x", limits.acc_lim_x);
        config_nh.getParam("acc_lim_y", limits.acc_lim_y);
        config_nh.getParam("acc_lim_theta", limits.acc_lim_theta);
        config_nh.getParam("acc_lim_trans", limits.acc_lim_trans);
        config_nh.getParam("xy_goal_tolerance", limits.xy_goal_tolerance);
        config_nh.getParam("yaw_goal_tolerance", limits.yaw_goal_tolerance);
        config_nh.getParam("prune_plan", limits.prune_plan);
        config_nh.getParam("trans_stopped_vel", limits.trans_stopped_vel);
        config_nh.getParam("theta_stopped_vel", limits.theta_stopped_vel);
        planner_util_.reconfigureCB(limits, false);
    */

    cp_ = boost::shared_ptr<CHOMPPlanner>(new CHOMPPlanner(name, &planner_util_));

    if (private_nh.getParam("odom_topic", odom_topic_))
    {
      odom_helper_.setOdomTopic(odom_topic_);
    }

    initialized_ = true;

    dsrv_ = new dynamic_reconfigure::Server<CHOMPPlannerConfig>(private_nh);
    dynamic_reconfigure::Server<CHOMPPlannerConfig>::CallbackType cb =
        boost::bind(&CHOMPPlannerROS::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
  }
  else
  {
    ROS_WARN("This planner has already been initialized, doing nothing.");
  }
}

bool CHOMPPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  // when we get a new plan, we also want to clear any latch we may have on goal tolerances
  latchedStopRotateController_.resetLatching();

  ROS_INFO("Got new plan");
  return cp_->setPlan(orig_global_plan);
}

bool CHOMPPlannerROS::isGoalReached()
{
  if (! isInitialized()) {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    if ( ! costmap_ros_->getRobotPose(current_pose_)) {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_)) {
      ROS_INFO("Goal reached");
      return true;
    } else {
      return false;
    }

}

bool CHOMPPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
{
  if (!costmap_ros_->getRobotPose(current_pose_))
  {
    ROS_ERROR("Could not get robot pose");
    return false;
  }
  std::vector<geometry_msgs::PoseStamped> transformed_plan;

  if (!planner_util_.getLocalPlan(current_pose_, transformed_plan))
  {
    ROS_ERROR("Could not get local plan");
    return false;
  }

  // if the global plan passed in is empty... we won't do anything
  if (transformed_plan.empty())
  {
    ROS_WARN_NAMED("chomp_planner", "Received an empty transformed plan.");
    return false;
  }
  ROS_DEBUG_NAMED("chomp_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

  // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
  cp_->updatePlanAndLocalCosts(current_pose_, transformed_plan, costmap_ros_->getRobotFootprint());

  // cost_map test
  //costmap_test();

  if (latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
  {
    ROS_DEBUG_NAMED("chomp_planner", "Position reached. Compute velocity commands to stop and rotate.");
    // publish an empty plan because we've reached our goal position

    
    std::vector<geometry_msgs::PoseStamped> local_plan;
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    publishGlobalPlan(transformed_plan);
    publishLocalPlan(local_plan);
    base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();

    return latchedStopRotateController_.computeVelocityCommandsStopRotate(
        cmd_vel, limits.getAccLimits(), cp_->getSimPeriod(), &planner_util_, odom_helper_, current_pose_,
        boost::bind(&CHOMPPlanner::checkTrajectory, cp_, _1, _2, _3));
  }
  else
  {
    bool isOk = chompComputeVelocityCommands(current_pose_, cmd_vel);
    if (isOk)
      publishGlobalPlan(transformed_plan);
    else
    {
      ROS_WARN_NAMED("chomp_planner", "DWA planner failed to produce path.");
      std::vector<geometry_msgs::PoseStamped> empty_plan;
      publishGlobalPlan(empty_plan);
    }
    return isOk;
  }
}

bool CHOMPPlannerROS::chompComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose,
                                                   geometry_msgs::Twist &cmd_vel)
{
  if (!isInitialized())
  {
    ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
    return false;
  }

  geometry_msgs::PoseStamped robot_vel;
  odom_helper_.getRobotVel(robot_vel);

  // ROS_DEBUG_NAMED("chomp_planner", "robot_vel: %f, %f, %f", robot_vel.pose.position.x, robot_vel.pose.position.y,
  // robot_vel.pose.orientation);

  geometry_msgs::Twist drive_cmd;
  std::string baseFrameID = costmap_ros_->getBaseFrameID();

  ros::Time last_time = ros::Time::now();

  std::vector<geometry_msgs::PoseStamped> local_plan =
      cp_->findBestPath(global_pose, baseFrameID, robot_vel, drive_cmd, &odom_helper_);

  ros::Time current_time = ros::Time::now();

  ROS_DEBUG_NAMED("chomp_planner", "choomp planner time used: %f", (current_time - last_time).toSec());

  publishLocalPlan(local_plan);

  cmd_vel.linear.x = drive_cmd.linear.x;
  cmd_vel.angular.z = drive_cmd.angular.z;

  return true;
}

void CHOMPPlannerROS::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path)
{
  base_local_planner::publishPlan(path, g_plan_pub_);
}

void CHOMPPlannerROS::publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path)
{
  base_local_planner::publishPlan(path, l_plan_pub_);
}

void CHOMPPlannerROS::reconfigureCB(CHOMPPlannerConfig &config, uint32_t level)
{
  ROS_DEBUG_NAMED("chomp_planner", "CHOMPPlannerROS config callback");
  if (setup_ && config.restore_defaults)
  {
    config = default_config_;
    config.restore_defaults = false;
  }
  if (!setup_)
  {
    default_config_ = config;
    setup_ = true;
  }

  // update generic local planner params
  base_local_planner::LocalPlannerLimits limits;
  limits.max_vel_trans = config.max_vel_trans;
  limits.min_vel_trans = config.min_vel_trans;
  limits.max_vel_x = config.max_vel_x;
  limits.min_vel_x = config.min_vel_x;
  limits.max_vel_y = config.max_vel_y;
  limits.min_vel_y = config.min_vel_y;
  limits.max_vel_theta = config.max_vel_theta;
  limits.min_vel_theta = config.min_vel_theta;
  limits.acc_lim_x = config.acc_lim_x;
  limits.acc_lim_y = config.acc_lim_y;
  limits.acc_lim_theta = config.acc_lim_theta;
  limits.acc_lim_trans = config.acc_lim_trans;
  limits.xy_goal_tolerance = config.xy_goal_tolerance;
  limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
  limits.prune_plan = config.prune_plan;
  limits.trans_stopped_vel = config.trans_stopped_vel;
  limits.theta_stopped_vel = config.theta_stopped_vel;
  planner_util_.reconfigureCB(limits, config.restore_defaults);

  // update dwa specific configuration
  cp_->reconfigure(config);
}

void CHOMPPlannerROS::costmap_test()
{
  costmap_2d::Costmap2D *costmap = costmap_ros_->getCostmap();
  int size_x = costmap->getSizeInCellsX();
  int size_y = costmap->getSizeInCellsY();
  ROS_DEBUG_NAMED("costmap_test", "size_x: %d, size_y: %d", size_x, size_y);
  for (int i = 0; i < size_x; ++i)
  {
    for (int j = 0; j < size_y; ++j)
    {
      std::cout << (unsigned int)costmap->getCost(i, j) << " " << setw(4);
    }
    std::cout << std::endl;
  }
}


};  // namespace chomp_local_planner
