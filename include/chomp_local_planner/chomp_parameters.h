/**
 * @file chomp_parameters.h
 * @author Kingsley
 * @brief 获取chomp相应参数，单例类
 * @version 0.1
 * @date 2019-04-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef CHOMP_PARAMETERS_H_
#define CHOMP_PARAMETERS_H_

#include <ros/ros.h>
#include <boost/noncopyable.hpp>
#include <eigen3/Eigen/Core>

static const unsigned int num_collision_points = 4;

class ChompParameters : boost::noncopyable
{
public:
  ~ChompParameters() {delete chomp_parameters;}

  static double getSmoothnessCostWeight() { return chomp_parameters->smoothness_cost_weight_; }
  static double getSmoothnessCostVelocity() { return chomp_parameters->smoothness_cost_velocity_; }
  static double getSmoothnessCostAcceleration() { return chomp_parameters->smoothness_cost_acceleration_; }
  static double getSmoothnessCostJerk() { return chomp_parameters->smoothness_cost_jerk_; }
  static double getRidgeFactor() { return chomp_parameters->ridge_factor_; }
  static double getLearningRate() { return chomp_parameters->learning_rate_; }
  static int getMaxIterations() { return chomp_parameters->max_iterations_; }
  static double getObstacleInflation() { return chomp_parameters->obstacle_inflation_radius_; }
  static double getRobotWidth() { return chomp_parameters->robot_width_; }
  static double getRobotLength() { return chomp_parameters->robot_length_; }
  static double getCollisionThreshold() { return chomp_parameters->collision_threshold_; }
  static double getObstacleCostWeight() { return chomp_parameters->obstacle_cost_weight_; }
  static int getMaxIterationsAfterCollision() { return chomp_parameters->max_iterations_after_collision_free_; }
  static double getPositionUpdateLimit() { return chomp_parameters->position_update_limit_; }
  static double getChangePotential() { return chomp_parameters->change_potential_; }
  static double getMaxVelX() { return chomp_parameters->max_vel_x_; }
  static double getMaxVelTheta() { return chomp_parameters->max_vel_theta_; }
  static int getSimPoints() { return chomp_parameters->sim_points_; }
private:
  ChompParameters();
  static ChompParameters *chomp_parameters;

public:
  //parameters
  double smoothness_cost_weight_;
  double obstacle_cost_weight_;
  double smoothness_cost_velocity_;
  double smoothness_cost_acceleration_;
  double smoothness_cost_jerk_;
  double ridge_factor_;
  double learning_rate_;
  int max_iterations_;
  
  double obstacle_inflation_radius_;

  double robot_width_;
  double robot_length_;

  double collision_threshold_;

  int max_iterations_after_collision_free_;

  double position_update_limit_;

  int change_potential_;

  double max_vel_x_;
  double max_vel_theta_;

  int sim_points_;

};

class RobotParameter : boost::noncopyable
{
public:
  Eigen::Vector2d vertex_[num_collision_points];

  inline static void getRobotVertex(int index, double &x, double &y)
  {
    ROS_ASSERT(index >= 0);
    ROS_ASSERT(index < num_collision_points);
    x = robot_parameter->vertex_[index](0, 0);
    y = robot_parameter->vertex_[index](1, 0);
  }

  ~RobotParameter() { delete robot_parameter; }
private:
  RobotParameter();
  static RobotParameter *robot_parameter;
  double robot_width_;
  double robot_length_;
  
};

ChompParameters* ChompParameters::chomp_parameters = new ChompParameters();

RobotParameter* RobotParameter::robot_parameter = new RobotParameter();


#endif  //  CHOMP_PARAMETERS_H_