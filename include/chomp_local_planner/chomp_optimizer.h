/**
 * @file chomp_optimizer.h
 * @author Kingsley
 * @brief 
 * @version 0.1
 * @date 2019-04-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef CHOMP_OPTIMIZER_H_
#define CHOMP_OPTIMIZER_H_

#include <chomp_local_planner/chomp_car_trajectory.h>
#include <chomp_local_planner/chomp_utils.h>
#include <boost/shared_ptr.hpp>
#include <chomp_local_planner/chomp_parameters.h>
#include <chomp_local_planner/stomp_cost.h>

#include <chomp_local_planner/chomp_obstacle_layer.h>

#include <base_local_planner/local_planner_util.h>

using namespace chomp_car_trajectory;
class ChompOptimizer
{
public:
  ChompOptimizer(ChompTrajectory *trajectory, base_local_planner::LocalPlannerUtil *planner_util);

  bool optimize();

private:
  ChompTrajectory *full_trajectory_;  //原始轨迹
  ChompTrajectory group_trajectory_; //主要操作的轨迹

  Eigen::MatrixXd original_group_trajectory_;
  Eigen::MatrixXd best_group_trajectory_;

  bool is_collision_free_;
  // 表示状态是否碰撞障碍物
  std::vector<bool> state_is_in_collision_;

  int num_collision_free_iterations_;

  int iteration_;

  int num_vars_free_; //可优化的中间点
  int num_vars_all_; //总点数

  //  可优化点的开始标号
  int free_vars_start_;
  int free_vars_end_;

  boost::shared_ptr<StompCost> position_costs_;

  Eigen::MatrixXd smoothness_increments_;
  Eigen::VectorXd smoothness_derivative_;

  Eigen::MatrixXd collision_increments_;

  double getSmoothnessCost();

  void calculateSmoothnessIncrements();

  void addIncrementsToTrajectory();

  void updateFullTrajectory();

  void performTrajectory();

  boost::shared_ptr<chomp_obstacle_layer::ChompObstacleLayer> obstacle_layer_;

  base_local_planner::LocalPlannerUtil *planner_util_;

  std::vector<double> collision_point_potential_;

  std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>
      collision_point_potential_gradient_[num_collision_points];
  
  double getCollisionCost();

  void calculateCollisionIncrements();

  template <typename Derived>
  void getJacobian(int trajectoryPoint, int vertex_index,
                   Eigen::MatrixBase<Derived> &jacobian);
  
  std::vector<double> robot_vertex_[num_collision_points];

  base_local_planner::LocalPlannerLimits limits_;
  double max_vel_x_;
  double min_vel_x_;
  double max_vel_theta_;
  double min_vel_theta_;
  double max_x_;
  double max_y_;
  double max_theta_;


  void getMotionLimits();

  void handleVelLimits();

  void handleLimits();

};


#endif