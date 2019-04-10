#ifndef CHOMP_PARAMETERS_H_
#define CHOMP_PARAMETERS_H_

#include <ros/ros.h>
#include <boost/noncopyable.hpp>

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
private:
  ChompParameters();
  static ChompParameters *chomp_parameters;

public:
  //parameters
  double smoothness_cost_weight_;
  double smoothness_cost_velocity_;
  double smoothness_cost_acceleration_;
  double smoothness_cost_jerk_;
  double ridge_factor_;
  double learning_rate_;
  int max_iterations_;
};

ChompParameters* ChompParameters::chomp_parameters = new ChompParameters();



#endif