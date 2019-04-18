#include <chomp_local_planner/chomp_utils.h>
#include <chomp_local_planner/stomp_cost.h>
#include <eigen3/Eigen/LU>
#include <iostream>

//#define LOGGER_DEBUG
#include "logger.h"


StompCost::StompCost(const ChompTrajectory &trajectory, const std::vector<double> &derivative_costs,
                     double ridge_factor)
{
  int num_vars_all = trajectory.getNumPoints();
  int num_vars_free = num_vars_all - 2 * (DIFF_RULE_LENGTH - 1);
  Eigen::MatrixXd diff_matrix = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);
  quad_cost_full_ = Eigen::MatrixXd::Zero(num_vars_all, num_vars_all);

  // 构造三个矩阵，其中quad_cost_full为平方差矩阵的和
  double multiplier = 1.0;
  for (unsigned int i = 0; i < derivative_costs.size(); i++)
  {
    multiplier *= trajectory.getDiscretization();
    diff_matrix = getDiffMatrix(num_vars_all, &DIFF_RULES[i][0]);
    quad_cost_full_ += (derivative_costs[i] * multiplier) * (diff_matrix.transpose() * diff_matrix);
  }

  quad_cost_full_ += Eigen::MatrixXd::Identity(num_vars_all, num_vars_all) * ridge_factor;

  quad_cost_ = quad_cost_full_.block(DIFF_RULE_LENGTH - 1, DIFF_RULE_LENGTH - 1, num_vars_free, num_vars_free);

  // 求逆
  quad_cost_inv_ = quad_cost_.inverse();

  MATRIX_DEBUG("quad_cost_full_", quad_cost_full_);
  MATRIX_DEBUG("quad_cost_", quad_cost_);
  MATRIX_DEBUG("quad_cost_inv_", quad_cost_inv_);
}

Eigen::MatrixXd StompCost::getDiffMatrix(int size, const double *diff_rule) const
{
  Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(size, size);
  for (int i = 0; i < size; i++)
  {
    for (int j = -DIFF_RULE_LENGTH / 2; j <= DIFF_RULE_LENGTH / 2; j++)
    {
      int index = i + j;
      if (index < 0)
        continue;
      if (index >= size)
        continue;
      matrix(i, index) = diff_rule[j + DIFF_RULE_LENGTH / 2];
    }
  }
  return matrix;
}

double StompCost::getMaxQuadCostInvValue() const
{
  return quad_cost_inv_.maxCoeff();
}

void StompCost::scale(double scale)
{
  double inv_scale = 1.0 / scale;
  quad_cost_inv_ *= inv_scale;
  quad_cost_ *= scale;
  quad_cost_full_ *= scale;
}
