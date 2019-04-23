/**
 * @file chomp_car_trajectory.cpp
 * @author Kingsley
 * @brief 
 * @version 0.1
 * @date 2019-04-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <chomp_local_planner/chomp_car_trajectory.h>

// namespace chomp_car_trajectory
namespace chomp_car_trajectory
{
/**
 * @brief Construct a new Chomp Trajectory:: Chomp Trajectory object
 * 
 * @param global_plan 全局路径
 * @param initial_yaw 
 * @param discretization 
 */
ChompTrajectory::ChompTrajectory(const vector<geometry_msgs::PoseStamped> &global_plan, double initial_yaw,
                                 double discretization)
  : num_points_(global_plan.size()), start_index_(1), end_index_(num_points_ - 2), discretization_(discretization)
{
  trajectory_.resize(num_points_, NUM_POSITION_PLAN);
  trajectory_ = Eigen::MatrixXd(num_points_, NUM_POSITION_PLAN);
  fillTrajectory(global_plan, initial_yaw);
}

/**
 * @brief Construct a new Chomp Trajectory:: Chomp Trajectory object
 *    输入一条轨迹，在源轨迹的头尾复制一段轨迹点
 * @param 源轨迹
 * @param diff_rule_length 在源轨迹的头尾添加的轨迹点数
 */
ChompTrajectory::ChompTrajectory(const ChompTrajectory &source_traj, int diff_rule_length)
  : discretization_(source_traj.discretization_)
{
  int start_extra = (diff_rule_length - 1) - source_traj.start_index_;
  int end_extra = (diff_rule_length - 1) - ((source_traj.num_points_ - 1) - source_traj.end_index_);

  num_points_ = source_traj.num_points_ + start_extra + end_extra;
  start_index_ = diff_rule_length - 1;
  end_index_ = (num_points_ - 1) - (diff_rule_length - 1);

  trajectory_.resize(num_points_, NUM_JOINTS_PLAN);
  trajectory_ = Eigen::MatrixXd(num_points_, NUM_JOINTS_PLAN);

  // 根据情况，复制轨迹
  for (int i = 0; i < num_points_; i++)
  {
    int source_traj_point = i - start_extra;
    if (source_traj_point < 0)
      source_traj_point = 0;
    if (source_traj_point >= source_traj.num_points_)
      source_traj_point = source_traj.num_points_ - 1;
    for (int j = 0; j < NUM_POSITION_PLAN; j++)
    {
      (*this)(i, j) = source_traj(source_traj_point, j);
    }
  }
}

/**
 * @brief 将ROS的全局路径填充到矩阵trajectory_中
 * 
 * @param global_plan ROS格式的全局路径
 * @param initial_yaw 
 */
void ChompTrajectory::fillTrajectory(const vector<geometry_msgs::PoseStamped> &global_plan, double initial_yaw)
{
  for (int i = 0; i < num_points_ - 1; ++i)
  {
    trajectory_(i, 0) = global_plan[i].pose.position.x;
    trajectory_(i, 1) = global_plan[i].pose.position.y;
    trajectory_(i, 2) = atan2(global_plan[i + 1].pose.position.y - global_plan[i].pose.position.y,
                              global_plan[i + 1].pose.position.x - global_plan[i].pose.position.x);
  }
  trajectory_(num_points_ - 1, 0) = global_plan[num_points_ - 1].pose.position.x;
  trajectory_(num_points_ - 1, 1) = global_plan[num_points_ - 1].pose.position.y;
  trajectory_(num_points_ - 1, 2) = trajectory_(num_points_ - 2, 2);

  trajectory_(0, 2) = initial_yaw;

  // std::cout<<trajectory_.matrix();
}


/**
 * @brief 将trajectory_赋值为group_trajectory
 * 
 * @param group_trajectory 
 */
void ChompTrajectory::updateFromGroupTrajectory(const ChompTrajectory &group_trajectory)
{
  int num_vars_free = end_index_ - start_index_ + 1;
  for (int i = 0; i < NUM_JOINTS_PLAN; i++)
  {
    trajectory_.block(start_index_, i, num_vars_free, 1) =
        group_trajectory.trajectory_.block(group_trajectory.start_index_, i, num_vars_free, 1);
  }
}

}  // namespace chomp_car_trajectory