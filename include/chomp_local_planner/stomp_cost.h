#ifndef STOMP_COST_H_
#define STOMP_COST_H_

#include <chomp_local_planner/chomp_car_trajectory.h>
#include <vector>

using namespace chomp_car_trajectory;

class StompCost
{
public:
  StompCost(const ChompTrajectory &trajectory, const std::vector<double> &derivative_costs, double ridge_factor = 0.0);

  //  得到矩阵quad_cost_inv_中的最大值
  double getMaxQuadCostInvValue() const;

  const Eigen::MatrixXd &getQuadraticCostInverse() const;

  //  得到关节轨迹的cost
  double getCost(Eigen::MatrixXd::ColXpr position_trajectory) const;

  //  将quad_cost_full_和quad_cost_放大为scale倍
  void scale(double scale);

  template <typename Derived>
  void getDerivative(Eigen::MatrixXd::ColXpr joint_trajectory, Eigen::MatrixBase<Derived> &derivative) const;

private:
  Eigen::MatrixXd quad_cost_full_;
  Eigen::MatrixXd quad_cost_;
  Eigen::MatrixXd quad_cost_inv_;

  Eigen::MatrixXd getDiffMatrix(int size, const double *diff_rule) const;
};

inline const Eigen::MatrixXd &StompCost::getQuadraticCostInverse() const
{
  return quad_cost_inv_;
}
inline double StompCost::getCost(Eigen::MatrixXd::ColXpr position_trajectory) const
{
  return position_trajectory.dot(quad_cost_full_ * position_trajectory);
}

template <typename Derived>
void StompCost::getDerivative(Eigen::MatrixXd::ColXpr position_trajectory, Eigen::MatrixBase<Derived> &derivative) const
{
  derivative = (quad_cost_full_ * (2.0 * position_trajectory));
}

#endif /* STOMP_COST_H_ */