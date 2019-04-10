#include <chomp_local_planner/chomp_parameters.h>

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
}


