#include <base_local_planner/goal_functions.h>
#include <chomp_local_planner/chomp_planner.h>
#include <cmath>

// for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/utils.h>
#include <chomp_local_planner/chomp_car_trajectory.h>
#include <tf/tf.h>
#include <chomp_local_planner/chomp_utils.h>

#include <chomp_local_planner/chomp_optimizer.h>

//#include <chomp_local_planner/chomp_obstacle.h>

#include <chomp_local_planner/chomp_obstacle_layer.h>

namespace chomp_local_planner
{
CHOMPPlanner::CHOMPPlanner(std::string name, base_local_planner::LocalPlannerUtil* planner_util)
  : planner_util_(planner_util)
{
  ros::NodeHandle private_nh("~/" + name);

  std::string controller_frequency_param_name;
  if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
  {
    sim_period_ = 0.05;
  }
  else
  {
    double controller_frequency = 0;
    private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
    if (controller_frequency > 0)
    {
      sim_period_ = 1.0 / controller_frequency;
    }
    else
    {
      ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
      sim_period_ = 0.05;
    }
  }
  ROS_INFO("Sim period is set to %.2f", sim_period_);

  preGlobalPath_pub = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
}

bool CHOMPPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  return planner_util_->setPlan(orig_global_plan);
}

void CHOMPPlanner::updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
                                           const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                           const std::vector<geometry_msgs::Point>& footprint_spec)
{
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i)
  {
    global_plan_[i] = new_plan[i]; 
  }
  
  footprint_ = footprint_spec;
}


void CHOMPPlanner::reconfigure(CHOMPPlannerConfig &config)
{
  boost::mutex::scoped_lock l(configuration_mutex_);

  ROS_DEBUG_NAMED("chomp_planner", "CHOMPPlanner reconfigure");
}
/*
bool CHOMPPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }*/

std::vector<geometry_msgs::PoseStamped> CHOMPPlanner::findBestPath(
          const geometry_msgs::PoseStamped& global_pose, 
          std::string baseFrameID, 
          const geometry_msgs::PoseStamped& global_vel, 
          std::vector<Vector2d>& drive_velocities, double yaw_initial)
{
  ROS_DEBUG_NAMED("chomp_planner", "CHOMPPlanner findBestPath");
  
  chomp_car_trajectory::ChompTrajectory trajectory(global_plan_, yaw_initial, sim_period_);

  ChompOptimizer chompOptimizer(&trajectory, planner_util_);
  chompOptimizer.optimize();

  /*chomp_obstacle::ChompObstacle chomp_obstacle(planner_util_->getCostmap(), 0.5);
  Eigen::MatrixXd local_trajectory = trajectory.getTrajectory();

  chomp_obstacle.viewCostMap();
  double wx,wy,distance;
  for(int i = 0; i < trajectory.getNumPoints(); ++i)
  {
    global_plan_[i].pose.position.x = local_trajectory(i, 0);
    global_plan_[i].pose.position.y = local_trajectory(i, 1);
    chomp_obstacle.getMinDistanceAndCoordinate(local_trajectory(i, 0), local_trajectory(i, 1), distance, wx, wy);
  }
  chomp_obstacle.viewCostMap();*/

  //chomp_obstacle_layer::ChompObstacleLayer obstacle_layer(planner_util_->getCostmap(), 0.3);
  
  //obstacle_layer.viewCostMap();
  //obstacle_layer.viewObstacleCells();

  Eigen::MatrixXd local_trajectory = trajectory.getTrajectory();
  double wx,wy,distance;
  for(int i = 0; i < trajectory.getNumPoints(); ++i)
  {
    global_plan_[i].pose.position.x = local_trajectory(i, 0);
    global_plan_[i].pose.position.y = local_trajectory(i, 1);
  }
  return global_plan_;
}


}  // namespace chomp_local_planner


