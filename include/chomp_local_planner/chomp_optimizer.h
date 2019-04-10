#ifndef CHOMP_OPTIMIZER_H_
#define CHOMP_OPTIMIZER_H_

#include <chomp_local_planner/chomp_car_trajectory.h>
#include <chomp_local_planner/chomp_utils.h>
#include <boost/shared_ptr.hpp>
#include <chomp_local_planner/chomp_parameters.h>
#include <chomp_local_planner/stomp_cost.h>

using namespace ChompCarTrajectory;
class ChompOptimizer
{
public:
  ChompOptimizer(ChompTrajectory *trajectory);

  bool optimize();

private:
  ChompTrajectory *full_trajectory_;  //原始轨迹
  ChompTrajectory group_trajectory_; //主要操作的轨迹

  Eigen::MatrixXd original_group_trajectory_;
  Eigen::MatrixXd best_group_trajectory_;

  int num_vars_free_; //可优化的中间点
  int num_vars_all_; //总点数

  //  可优化点的开始标号
  int free_vars_start_;
  int free_vars_end_;

  boost::shared_ptr<StompCost> position_costs_;

  Eigen::MatrixXd smoothness_increments_;
  Eigen::VectorXd smoothness_derivative_;

  double getSmoothnessCost();

  void calculateSmoothnessIncrements();

  void addIncrementsToTrajectory();

  void updateFullTrajectory();
};


#endif