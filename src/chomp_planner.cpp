/**
 * @file chomp_planner.cpp
 * @author Kingsley
 * @brief 
 * @version 0.1
 * @date 2019-04-23
 * 
 * @copyright Copyright (c) 2019
 * 
 */

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
CHOMPPlanner::CHOMPPlanner(std::string name, base_local_planner::LocalPlannerUtil *planner_util)
    : planner_util_(planner_util),
      obstacle_costs_(planner_util->getCostmap()),
      path_costs_(planner_util->getCostmap()),
      goal_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      goal_front_costs_(planner_util->getCostmap(), 0.0, 0.0, true),
      alignment_costs_(planner_util->getCostmap())

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

  //dwa 控制停止部分
  std::vector<base_local_planner::TrajectoryCostFunction *> critics;
  oscillation_costs_.resetOscillationFlags();
  bool sum_scores;
  private_nh.param("sum_scores", sum_scores, false);
  obstacle_costs_.setSumScores(sum_scores);
  goal_front_costs_.setStopOnFailure( false );
  alignment_costs_.setStopOnFailure( false );

  critics.push_back(&oscillation_costs_); // discards oscillating motions (assisgns cost -1)
  critics.push_back(&obstacle_costs_);    // discards trajectories that move into obstacles
  critics.push_back(&goal_front_costs_);  // prefers trajectories that make the nose go towards (local) nose goal
  critics.push_back(&alignment_costs_);   // prefers trajectories that keep the robot nose on nose path
  critics.push_back(&path_costs_);        // prefers trajectories on global path
  critics.push_back(&goal_costs_);        // prefers trajectories that go towards (local) goal, based on wave propagation
  critics.push_back(&twirling_costs_);    // optionally prefer trajectories that don't spin
  // trajectory generators
  std::vector<base_local_planner::TrajectorySampleGenerator *> generator_list;
  generator_list.push_back(&generator_);

  scored_sampling_planner_ = base_local_planner::SimpleScoredSamplingPlanner(generator_list, critics);

  private_nh.param("cheat_factor", cheat_factor_, 1.0);
}

bool CHOMPPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
{
  return planner_util_->setPlan(orig_global_plan);
}

void CHOMPPlanner::updatePlanAndLocalCosts(const geometry_msgs::PoseStamped &global_pose,
                                           const std::vector<geometry_msgs::PoseStamped> &new_plan,
                                           const std::vector<geometry_msgs::Point> &footprint_spec)
{
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i)
  {
    global_plan_[i] = new_plan[i];
  }

  obstacle_costs_.setFootprint(footprint_spec);

  // costs for going away from path
  path_costs_.setTargetPoses(global_plan_);

  // costs for not going towards the local goal as much as possible
  goal_costs_.setTargetPoses(global_plan_);

  // alignment costs
  geometry_msgs::PoseStamped goal_pose = global_plan_.back();

  Eigen::Vector3f pos(global_pose.pose.position.x, global_pose.pose.position.y, tf2::getYaw(global_pose.pose.orientation));
  double sq_dist =
      (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
      (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);

  // we want the robot nose to be drawn to its final position
  // (before robot turns towards goal orientation), not the end of the
  // path for the robot center. Choosing the final position after
  // turning towards goal orientation causes instability when the
  // robot needs to make a 180 degree turn at the end
  std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
  double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
  front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
                                             forward_point_distance_ * cos(angle_to_goal);
  front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
                                                                                            sin(angle_to_goal);

  goal_front_costs_.setTargetPoses(front_global_plan);

  // keeping the nose on the path
  if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_)
  {
    alignment_costs_.setScale(pdist_scale_);
    // costs for robot being aligned with path (nose on path, not ju
    alignment_costs_.setTargetPoses(global_plan_);
  }
  else
  {
    // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
    alignment_costs_.setScale(0.0);
  }
}

void CHOMPPlanner::reconfigure(CHOMPPlannerConfig &config)
{
  boost::mutex::scoped_lock l(configuration_mutex_);

  generator_.setParameters(
      config.sim_time,
      config.sim_granularity,
      config.angular_sim_granularity,
      config.use_dwa,
      sim_period_);

  double resolution = planner_util_->getCostmap()->getResolution();
  pdist_scale_ = resolution * config.path_distance_bias;
  // pdistscale used for both path and alignment, set  forward_point_distance to zero to discard alignment
  path_costs_.setScale(pdist_scale_);
  alignment_costs_.setScale(pdist_scale_);

  gdist_scale_ = resolution * config.goal_distance_bias;
  goal_costs_.setScale(gdist_scale_);
  goal_front_costs_.setScale(gdist_scale_);

  occdist_scale_ = config.occdist_scale;
  obstacle_costs_.setScale(occdist_scale_);

  stop_time_buffer_ = config.stop_time_buffer;
  oscillation_costs_.setOscillationResetDist(config.oscillation_reset_dist, config.oscillation_reset_angle);
  forward_point_distance_ = config.forward_point_distance;
  goal_front_costs_.setXShift(forward_point_distance_);
  alignment_costs_.setXShift(forward_point_distance_);

  // obstacle costs can vary due to scaling footprint feature
  obstacle_costs_.setParams(config.max_vel_trans, config.max_scaling_factor, config.scaling_speed);

  twirling_costs_.setScale(config.twirling_scale);

  int vx_samp, vy_samp, vth_samp;
  vx_samp = config.vx_samples;
  vy_samp = config.vy_samples;
  vth_samp = config.vth_samples;

  if (vx_samp <= 0)
  {
    ROS_WARN("You've specified that you don't want any samples in the x dimension. We'll at least assume that you want to sample one value... so we're going to set vx_samples to 1 instead");
    vx_samp = 1;
    config.vx_samples = vx_samp;
  }

  if (vy_samp <= 0)
  {
    ROS_WARN("You've specified that you don't want any samples in the y dimension. We'll at least assume that you want to sample one value... so we're going to set vy_samples to 1 instead");
    vy_samp = 1;
    config.vy_samples = vy_samp;
  }

  if (vth_samp <= 0)
  {
    ROS_WARN("You've specified that you don't want any samples in the th dimension. We'll at least assume that you want to sample one value... so we're going to set vth_samples to 1 instead");
    vth_samp = 1;
    config.vth_samples = vth_samp;
  }

  vsamples_[0] = vx_samp;
  vsamples_[1] = vy_samp;
  vsamples_[2] = vth_samp;

  ROS_DEBUG_NAMED("chomp_planner", "CHOMPPlanner reconfigure");
}

bool CHOMPPlanner::checkTrajectory(
    Eigen::Vector3f pos,
    Eigen::Vector3f vel,
    Eigen::Vector3f vel_samples)
{
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
  if (cost >= 0)
  {
    return true;
  }
  ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

  //otherwise the check fails
  return false;
  }


/**
 * @brief 得到优化后的局部路径和规划的速度
 * 
 * @param global_pose 
 * @param baseFrameID 
 * @param global_vel 
 * @param drive_vel  输出规划的速度
 * @param odom_helper  
 * @return std::vector<geometry_msgs::PoseStamped> 返回优化后的路径
 */
std::vector<geometry_msgs::PoseStamped> CHOMPPlanner::findBestPath(
          const geometry_msgs::PoseStamped& global_pose, 
          std::string baseFrameID, 
          const geometry_msgs::PoseStamped& global_vel, 
          geometry_msgs::Twist &drive_vel, base_local_planner::OdometryHelperRos *odom_helper)
{
  chomp_car_trajectory::ChompTrajectory trajectory(global_plan_, odom_helper->getOdomYaw(), sim_period_);

  ChompOptimizer chompOptimizer(&trajectory, planner_util_);
  chompOptimizer.optimize();

  getCmdVel(trajectory.getTrajectory(), odom_helper, drive_vel);

  Eigen::MatrixXd local_trajectory = trajectory.getTrajectory();

  for(int i = 0; i < trajectory.getNumPoints(); ++i)
  {
    global_plan_[i].pose.position.x = local_trajectory(i, 0);
    global_plan_[i].pose.position.y = local_trajectory(i, 1);
  }
  return global_plan_;
}
/**
 * @brief 从给定的轨迹中计算出初速度
 * 
 * @param position_trajectory 
 * @param odom_helper 
 * @param cmd_vel 
 */
void CHOMPPlanner::getCmdVel(Eigen::MatrixXd &position_trajectory, base_local_planner::OdometryHelperRos *odom_helper, geometry_msgs::Twist &cmd_vel)
{
  static bool stop_to_rotate = false;

  double init_vel_x = vel_last_time_.linear.x;
  double init_vel_yaw = vel_last_time_.angular.z;

  if(position_trajectory.rows() <= 5)
  {
    //should perform deceleration motion
    cmd_vel.linear.x = init_vel_x;
    cmd_vel.angular.z = init_vel_yaw;

    vel_last_time_.linear.x = vel_last_time_.angular.z = 0;
    return;
  }

  int sim_points = ChompParameters::getSimPoints();
  if(sim_points >= position_trajectory.rows())
    sim_points = position_trajectory.rows() / 2;
  
  double init_yaw = odom_helper->getOdomYaw();
  double final_yaw; 

  double delta_x = position_trajectory(sim_points, 0) - position_trajectory(0, 0);
  double delta_y = position_trajectory(sim_points, 1) - position_trajectory(0, 1);

  final_yaw = atan2(delta_y, delta_x);

  double linear_vel, angular_vel;

  for (; ; ++sim_points)
  {
    linear_vel = sqrt(((delta_x * delta_x) + (delta_y * delta_y)) / (double(sim_points) * sim_period_ * double(sim_points) * sim_period_));

    angular_vel = (final_yaw - init_yaw) / ((double)sim_points * sim_period_);

    if(fabs(linear_vel) <= ChompParameters::getMaxVelX() && fabs(angular_vel) <= 1.0)
      break;
  }

  if(fabs(final_yaw - init_yaw) > 0.5)
  {
    stop_to_rotate = true;
    ROS_INFO("consider rotate. yaw: %f, final_yaw: %f, init_yaw: %f", final_yaw - init_yaw, final_yaw / 3.1415 * 180.0, init_yaw / 3.1415 * 180.0);
  }

  if(stop_to_rotate)
  {
    if(fabs(final_yaw - init_yaw) < 0.1)
      stop_to_rotate = false;
  }

  if(stop_to_rotate)
  {
    linear_vel = 0;
    ROS_INFO("consider rotate. yaw: %f", final_yaw - init_yaw);
  }
  else
  {
    if(fabs(final_yaw - init_yaw) > 0.2)
    {
      ROS_INFO("consider deceleration. yaw: %f", final_yaw - init_yaw);
      if(linear_vel >= 0.01)
        linear_vel -= 0.01;
    }
  }

  cmd_vel.linear.x = linear_vel;
  cmd_vel.angular.z = angular_vel;

  vel_last_time_.linear.x = linear_vel;
  vel_last_time_.angular.z = angular_vel;

  ROS_DEBUG_NAMED("chomp_planner", "chomp planner get cmd_vel: %f, %f", linear_vel, angular_vel);
}

}  // namespace chomp_local_planner


