/**
 * @file chomp_car_trajectory.h
 * @author Kingsley
 * @brief 
 * @version 0.1
 * @date 2019-04-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef CHOMP_CAR_TRAJECTORY_H_
#define CHOMP_CAR_TRAJECTORY_H_

#include <chomp_local_planner/chomp_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
using namespace std;

namespace chomp_car_trajectory
{
class ChompTrajectory
{
public:
  ChompTrajectory(const vector<geometry_msgs::PoseStamped> &global_plan, double initial_yaw, double discretization);
  ChompTrajectory(const ChompTrajectory &source_traj, int diff_rule_length);

  double &operator()(int traj_point, int position);
  double operator()(int traj_point, int position) const;

  double getDiscretization() const;

  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);

  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreeTrajectoryBlock();

  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> getFreePositionTrajectoryBlock(int position);

  void updateFromGroupTrajectory(const ChompTrajectory &group_trajectory);

  int getNumFreePoints() const;
  int getNumPoints() const;
  int getStartIndex() const;
  int getEndIndex() const;
  void setStartIndex(int start_index)
  {
    if (start_index >= 1 && start_index <= num_points_ - 2)
      start_index_ = start_index;
  }
  void setEndIndex(int end_index)
  {
    if (end_index >= 1 && end_index <= num_points_ - 2)
      end_index_ = end_index;
  }

  Eigen::MatrixXd &getTrajectory();
  Eigen::MatrixXd::ColXpr getPositionTrajectory(int position);

private:
  Eigen::MatrixXd trajectory_;  //轨迹
  int num_points_;
  int start_index_;
  int end_index_;
  double discretization_;
  void fillTrajectory(const vector<geometry_msgs::PoseStamped> &global_plan, double initial_yaw);
};

/**
 * @brief 获取轨迹的点，重载操作符号()
 * 
 * @param traj_point 轨迹index
 * @param position x, y, theta
 * @return double& 返回对应点引用
 */
inline double &ChompTrajectory::operator()(int traj_point, int position)
{
  return trajectory_(traj_point, position);
}

inline double ChompTrajectory::operator()(int traj_point, int position) const
{
  return trajectory_(traj_point, position);
}

//获取轨迹矩阵
inline Eigen::MatrixXd &ChompTrajectory::getTrajectory()
{
  return trajectory_;
}

//获取轨迹第traj_point行的值，即某一点的(x, y, theta)
inline Eigen::MatrixXd::RowXpr ChompTrajectory::getTrajectoryPoint(int traj_point)
{
  return trajectory_.row(traj_point);
}

//获取源轨迹点数
inline int ChompTrajectory::getNumFreePoints() const
{
  return (end_index_ - start_index_) + 1;
}

//获取源轨迹
inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic> ChompTrajectory::getFreeTrajectoryBlock()
{
  return trajectory_.block(start_index_, 0, getNumFreePoints(), NUM_POSITION_PLAN);
}

//获取源轨迹第position列
inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
ChompTrajectory::getFreePositionTrajectoryBlock(int position)
{
  return trajectory_.block(start_index_, position, getNumFreePoints(), 1);
}

inline int ChompTrajectory::getNumPoints() const
{
  return num_points_;
}

inline int ChompTrajectory::getStartIndex() const
{
  return start_index_;
}

inline int ChompTrajectory::getEndIndex() const
{
  return end_index_;
}

//获取轨迹矩阵第position列
inline Eigen::MatrixXd::ColXpr ChompTrajectory::getPositionTrajectory(int position)
{
  return trajectory_.col(position);
}

inline double ChompTrajectory::getDiscretization() const
{
  return discretization_;
}

}  // namespace chomp_car_trajectory

#endif  // CHOMP_CAR_TRAJECTORY_H_