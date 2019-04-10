#ifndef CHOMP_CAR_TRAJECTORY_H_
#define CHOMP_CAR_TRAJECTORY_H_

#include <chomp_local_planner/chomp_utils.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
using namespace std;

namespace ChompCarTrajectory
{
class ChompTrajectory
{
public:
  ChompTrajectory(vector<geometry_msgs::PoseStamped> &global_plan, double initial_yaw, double discretization);
  ChompTrajectory(const ChompTrajectory &source_traj, int diff_rule_length);

  double &operator()(int traj_point, int position);
  double operator()(int traj_point, int position) const;

  double getDiscretization() const;

  Eigen::MatrixXd::RowXpr getTrajectoryPoint(int traj_point);

  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
  getFreeTrajectoryBlock();

  Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
  getFreePositionTrajectoryBlock(int position);

  void updateFromGroupTrajectory(const ChompTrajectory &group_trajectory);

  int getNumFreePoints() const;
  int getNumPoints() const;
  int getStartIndex() const;
  int getEndIndex() const;
  void setStartIndex(int start_index) {
    if (start_index >= 1 && start_index <= num_points_ - 2)
      start_index_ = start_index;
  }
  void setEndIndex(int end_index) {
    if (end_index >= 1 && end_index <= num_points_ - 2)
      end_index_ = end_index;
  }

  Eigen::MatrixXd &getTrajectory();
  Eigen::MatrixXd::ColXpr getPositionTrajectory(int position);
private:
  Eigen::MatrixXd trajectory_; //真实的轨迹
  int num_points_;
  int start_index_;
  int end_index_;
  double discretization_;
  void fillTrajectory(vector<geometry_msgs::PoseStamped> &global_plan, double initial_yaw);
};


inline double &ChompTrajectory::operator()(int traj_point, int position) {
  return trajectory_(traj_point, position);
}

inline double ChompTrajectory::operator()(int traj_point, int position) const {
  return trajectory_(traj_point, position);
}

inline Eigen::MatrixXd & ChompTrajectory::getTrajectory()
{
  return trajectory_;
}

inline Eigen::MatrixXd::RowXpr
ChompTrajectory::getTrajectoryPoint(int traj_point) {
  return trajectory_.row(traj_point);
}

inline int ChompTrajectory::getNumFreePoints() const {
  return (end_index_ - start_index_) + 1;
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
ChompTrajectory::getFreeTrajectoryBlock() {
  return trajectory_.block(start_index_, 0, getNumFreePoints(),
                           NUM_POSITION_PLAN);
}

inline Eigen::Block<Eigen::MatrixXd, Eigen::Dynamic, Eigen::Dynamic>
ChompTrajectory::getFreePositionTrajectoryBlock(int position) {
  return trajectory_.block(start_index_, position, getNumFreePoints(), 1);
}

inline int ChompTrajectory::getNumPoints() const { return num_points_; }


inline int ChompTrajectory::getStartIndex() const { return start_index_; }

inline int ChompTrajectory::getEndIndex() const { return end_index_; }

inline Eigen::MatrixXd::ColXpr ChompTrajectory::getPositionTrajectory(int position)
{
  return trajectory_.col(position);
}

inline double ChompTrajectory::getDiscretization() const {
  return discretization_;
}

}  // namespace ChompCarTrajectory

#endif  // CHOMP_CAR_TRAJECTORY_H_