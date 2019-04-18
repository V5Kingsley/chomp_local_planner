/**
 * @file chomp_parameters.cpp
 * @author Kingsley
 * @brief 
 * @version 0.1
 * @date 2019-04-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <chomp_local_planner/chomp_parameters.h>
#include <cmath>

ChompParameters::ChompParameters()
{
  ros::NodeHandle nh("~ChompParameters");
  nh.param<double>("smoothness_cost_weight", smoothness_cost_weight_, 10.0);
  nh.param<double>("smoothness_cost_velocity", smoothness_cost_velocity_, 0.0);
  nh.param<double>("smoothness_cost_acceleration", smoothness_cost_acceleration_, 1.0);
  nh.param<double>("smoothness_cost_jerk", smoothness_cost_jerk_, 0.0);
  nh.param<double>("ridge_factor", ridge_factor_, 0.0);
  nh.param<double>("learning_rate", learning_rate_, 0.2);
  nh.param<int>("max_iterations", max_iterations_, 800);
  nh.param<double>("obstacle_inflation_radius", obstacle_inflation_radius_, 1.0);
  nh.param<double>("robot_width", robot_width_, 0.5);
  nh.param<double>("robot_length", robot_length_, 0.75);
  nh.param<double>("collision_threshold", collision_threshold_, 0.001);
  nh.param<double>("obstacle_cost_weight", obstacle_cost_weight_, 10.0);
  nh.param<int>("max_iterations_after_collision_free", max_iterations_after_collision_free_, 1);
  nh.param<double>("position_update_limit", position_update_limit_, 0.15);

}

RobotParameter::RobotParameter()
  : robot_width_(ChompParameters::getRobotWidth()), robot_length_(ChompParameters::getRobotLength())
{
  vertex_[0] << robot_length_ / 2.0, robot_width_ / 2.0;

  vertex_[1] << -robot_length_ / 2.0, robot_width_ / 2.0;

  vertex_[2] << -robot_length_ / 2.0, -robot_width_ / 2.0;

  vertex_[3] << robot_length_ / 2.0, -robot_width_ / 2.0;

  ROS_INFO("robot footprint: (%f, %f), (%f, %f), (%f, %f), (%f, %f),", vertex_[0][0], vertex_[0][1], vertex_[1][0], vertex_[1][1], vertex_[2][0], vertex_[2][1], vertex_[3][0], vertex_[3][1]);

}


