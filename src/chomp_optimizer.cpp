#include <chomp_local_planner/chomp_optimizer.h>

#define LOGGER_DEBUG
#include "logger.h"

ChompOptimizer::ChompOptimizer(ChompTrajectory *trajectory, base_local_planner::LocalPlannerUtil *planner_util)
	: full_trajectory_(trajectory), 
		group_trajectory_(*trajectory, DIFF_RULE_LENGTH), 
		planner_util_(planner_util),
		num_collision_free_iterations_(0)
{
	num_vars_free_ = group_trajectory_.getNumFreePoints();
	num_vars_all_ = group_trajectory_.getNumPoints();

	free_vars_start_ = group_trajectory_.getStartIndex();
	free_vars_end_ = group_trajectory_.getEndIndex();

	std::vector<double> derivative_costs(3);
	derivative_costs[0] = ChompParameters::getSmoothnessCostVelocity();
	derivative_costs[1] = ChompParameters::getSmoothnessCostAcceleration();
	derivative_costs[2] = ChompParameters::getSmoothnessCostJerk();
	position_costs_.reset(new StompCost(group_trajectory_, derivative_costs, ChompParameters::getRidgeFactor()));

	double cost_scale = position_costs_->getMaxQuadCostInvValue();
	double max_cost_scale = 0.0;
	if (max_cost_scale < cost_scale)
		max_cost_scale = cost_scale;
	//  放缩joint_costs_矩阵，类似于SHOMP文章第三页算法中Precompute的第三行
	position_costs_->scale(max_cost_scale);

	// 对矩阵分配内存
	smoothness_increments_ = Eigen::MatrixXd::Zero(num_vars_free_, NUM_POSITION_PLAN);
	smoothness_derivative_ = Eigen::VectorXd::Zero(num_vars_all_);
	collision_increments_ =
      Eigen::MatrixXd::Zero(num_vars_free_, NUM_POSITION_PLAN);

	is_collision_free_ = false;
	state_is_in_collision_.resize(num_vars_all_);

	for(int i = 0; i < num_collision_points; ++i)
	{
		collision_point_potential_gradient_[i].resize(num_vars_all_);
	}
	collision_point_potential_.resize(num_vars_all_);

	obstacle_layer_ = boost::shared_ptr<chomp_obstacle_layer::ChompObstacleLayer>(
			new chomp_obstacle_layer::ChompObstacleLayer(planner_util_->getCostmap(), 0.3));
	
  for(int i = 0; i < num_collision_points; ++i)
  {
    robot_vertex_[i].resize(2);
    RobotParameter::getRobotVertex(i, robot_vertex_[i][0], robot_vertex_[i][1]);
  }

	getMotionLimits();
}

bool ChompOptimizer::optimize()
{
	original_group_trajectory_ = group_trajectory_.getTrajectory();
	best_group_trajectory_ = group_trajectory_.getTrajectory();
	double best_group_trajectory_cost = 10000;

	int max_iteration = ChompParameters::getMaxIterations();
	int num_collision_free_iterations = 0;

	MATRIX_DEBUG("group_trajectory before iteration", group_trajectory_.getTrajectory());
	
	for (iteration_ = 0; iteration_ < max_iteration; ++iteration_)
	{
		ROS_DEBUG_NAMED("chomp_optimizer", "iteration[%d]", iteration_);

		performTrajectory();	//获取轨迹的collision_point_potential_, collision_point_potential_gradient_

		double smoothCost = getSmoothnessCost();
		double collisionCost = getCollisionCost();
		double cost = smoothCost + collisionCost;

		ROS_DEBUG_NAMED("chomp_optimizer", "cost: %f", cost);
		if (cost < best_group_trajectory_cost)
		{
			best_group_trajectory_ = group_trajectory_.getTrajectory();
			best_group_trajectory_cost = cost;
		}

		if(is_collision_free_ == true)
		{
			num_collision_free_iterations_++;
			if(num_collision_free_iterations_ >= ChompParameters::getMaxIterationsAfterCollision())
				break;
		}
		else
		{
			num_collision_free_iterations_ = 0;
		}
		

		calculateCollisionIncrements();

		calculateSmoothnessIncrements();

		addIncrementsToTrajectory();

		//handleVelLimits();
		handleLimits();
		/*
				// 用于生成txt文档
				int iteration_skip_num = 1;
				if (true && iteration % iteration_skip_num == 0) {
					updateFullTrajectory();
					unsigned int num_points = full_trajectory_->getNumPoints();
					std::stringstream ss;
					ss << int((iteration + 0.01) / iteration_skip_num);
					std::string file_name = "path/path" + ss.str() + ".txt";
					std::ofstream outFile(file_name);
					for (unsigned int i = 0; i < num_points; ++i) {
						for (unsigned int j = 0; j < NUM_JOINTS_PLAN; ++j) {
							outFile << full_trajectory_->getTrajectoryPoint(i)(j) << " ";
						}
						outFile << std::endl;
					}
					outFile.close();
				}*/
	//	MATRIX_DEBUG("best_group_trajectory", best_group_trajectory_);
		//MATRIX_DEBUG("group_trajectory", group_trajectory_.getTrajectory());
	}

	if(is_collision_free_)
	{
		group_trajectory_.getTrajectory() = best_group_trajectory_;
		updateFullTrajectory();
		ROS_DEBUG_NAMED("chomp_optimizer", "chomp optimizer succeeded. iterations: %d", iteration_);
	}
	else
	{
		group_trajectory_.getTrajectory() = original_group_trajectory_;
    updateFullTrajectory();
		ROS_WARN_NAMED("chomp_optimizer", "chomp optimizer failed.");
	}
	
	//obstacle_layer_->viewObstacleCells();
	//obstacle_layer_->viewCostMap();

//	MATRIX_DEBUG("After iteration, the trajectory becomes: ", full_trajectory_->getTrajectory());
}

double ChompOptimizer::getSmoothnessCost()
{
	double smoothness_cost = 0.0;
	for (unsigned int i = 0; i < NUM_POSITION_PLAN; ++i)
	{
		smoothness_cost += position_costs_->getCost(group_trajectory_.getPositionTrajectory(i));
	}
	return ChompParameters::getSmoothnessCostWeight() * smoothness_cost;
}

void ChompOptimizer::calculateSmoothnessIncrements()
{
	for (unsigned int i = 0; i < NUM_POSITION_PLAN; ++i)
	{
		position_costs_->getDerivative(group_trajectory_.getPositionTrajectory(i), smoothness_derivative_);
		smoothness_increments_.col(i) = -smoothness_derivative_.segment(
				group_trajectory_.getStartIndex(), num_vars_free_);	// segment(i,n)获取第i个元素开始的n个元素
	}
}

void ChompOptimizer::addIncrementsToTrajectory()
{
	Eigen::MatrixXd final_increments;
	final_increments = Eigen::MatrixXd::Zero(num_vars_free_, NUM_POSITION_PLAN);

	for (unsigned i = 0; i < NUM_POSITION_PLAN; ++i)
	{
		final_increments.col(i) = ChompParameters::getLearningRate() *
															(position_costs_->getQuadraticCostInverse() *
															 (ChompParameters::getSmoothnessCostWeight() * smoothness_increments_.col(i) + 
															 	ChompParameters::getObstacleCostWeight() * collision_increments_.col(i)));
		double scale = 1.0;
		double max = final_increments.col(i).maxCoeff();
    double min = final_increments.col(i).minCoeff();
		double max_scale = ChompParameters::getPositionUpdateLimit() / fabs(max);
		double min_scale = ChompParameters::getPositionUpdateLimit() / fabs(min);
		if (max_scale < scale)
      scale = max_scale;
    if (min_scale < scale)
      scale = min_scale;
		group_trajectory_.getFreeTrajectoryBlock().col(i) += scale * final_increments.col(i);
	}
}

void ChompOptimizer::updateFullTrajectory()
{
	full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}

void ChompOptimizer::performTrajectory()
{
	ROS_DEBUG_NAMED("chomp_optimizer", "chomp_optimizer : perform trajectory");

	int start = free_vars_start_;
	int end = free_vars_end_;
	// 第一次迭代要全部状态都要计算相关量，从第二次开始，只计算中间可优化的量。
	if (iteration_ == 0)
	{
		start = 0;
		end = num_vars_all_ - 1;
	}
	is_collision_free_ = true;

	for (int i = start; i <= end; ++i)
	{
		std::vector<double> state;
		for (unsigned int j = 0; j < NUM_POSITION_PLAN; j++)
		{
			state.push_back(group_trajectory_(i, j));
		}

		double distance;
		obstacle_layer_->getPotential(state, collision_point_potential_[i], collision_point_potential_gradient_, i, distance);

		//ROS_DEBUG_NAMED("chomp_optimizer", "state: (%f, %f, %f), distance: %f, potential: %f", state[0], state[1], state[2], distance, collision_point_potential_[i]);

		if (distance < ChompParameters::getCollisionThreshold())
		{
			state_is_in_collision_[i] = true;
			is_collision_free_ = false;
		}
	}

  ROS_DEBUG_NAMED("chomp_optimizer", "chomp_optimizer : perform trajectory over");
}

double ChompOptimizer::getCollisionCost()
{
	double collision_cost = 0.0;

	for (int i = free_vars_start_; i <= free_vars_end_; i++)
	{
		collision_cost += collision_point_potential_[i];
	}

	return ChompParameters::getObstacleCostWeight() * collision_cost;
}

void ChompOptimizer::calculateCollisionIncrements()
{
	int startPoint = free_vars_start_;
  int endPoint = free_vars_end_;

	double potential;
	Eigen::Vector3d potential_gradient;
	Eigen::MatrixXd jacobian;
	jacobian = Eigen::MatrixXd::Zero(NUM_POSITION_PLAN, NUM_POSITION_PLAN);
	collision_increments_.setZero(num_vars_free_, NUM_POSITION_PLAN);

	for(int i = startPoint; i <= endPoint; ++i)
	{
		potential = collision_point_potential_[i];
		if(potential < 0.0001)
			continue;
		
		for(unsigned int j = 0; j < num_collision_points; ++j)
		{
			getJacobian(i, j, jacobian);
			potential_gradient = - collision_point_potential_gradient_[j][i];
			
			collision_increments_.row(i - free_vars_start_).transpose() -=
          jacobian.transpose() * potential_gradient;
		}
		

	}
}

template <typename Derived>
void ChompOptimizer::getJacobian(int trajectory_point, int j,
                                 Eigen::MatrixBase<Derived> &jacobian)
{
  jacobian = Eigen::MatrixXd::Identity(3, 3);
  double theta = group_trajectory_(trajectory_point, 2);
  jacobian(0, 2) = - robot_vertex_[j][0] * sin(theta) - robot_vertex_[j][1] * cos(theta);
  jacobian(1, 2) = robot_vertex_[j][0] * cos(theta) - robot_vertex_[j][1] * sin(theta);
}


void ChompOptimizer::getMotionLimits()
{
	/*limits_ = planner_util_->getCurrentLimits();
	max_vel_x_ = limits_.max_vel_x;
	min_vel_x_ = limits_.min_vel_x;
	max_vel_theta_ = limits_.max_vel_theta;
	min_vel_theta_ = limits_.min_vel_theta;*/

	max_vel_x_ = 0.22;
	max_vel_theta_ = 1.57;

	double discretization = full_trajectory_->getDiscretization();
	max_x_ = max_vel_x_ * discretization;
	max_theta_ = max_vel_theta_ * discretization;
	max_y_ = max_x_ * sin(max_theta_);


	ROS_DEBUG_NAMED("chomp_optimizer", "discretization: %f, max_x: %f, max_y: %f, max_theta: %f", discretization, max_x_, max_y_, max_theta_);
	
}

void ChompOptimizer::handleVelLimits()
{
	std::vector<double> position_limit{max_x_, max_y_, max_theta_};
	bool violation = false;
	int count = 0;
	for (unsigned int position_i = 2; position_i < NUM_POSITION_PLAN; ++position_i)
	{
		do
		{
			double max_abs_violation = 1e-6;
			double max_violation = 0.0;
			violation = false;
			int max_violation_index = 0;
			for (int i = free_vars_start_ + 1; i <= free_vars_end_; i++)
			{
				double amount = 0.0;
				double absolute_amount = 0.0;
				if (group_trajectory_(i, position_i) - group_trajectory_(i - 1, position_i) > position_limit[position_i])
				{
					amount = group_trajectory_(i - 1, position_i) + position_limit[position_i] - group_trajectory_(i, position_i);
					absolute_amount = fabs(amount);
				}
				else if (group_trajectory_(i, position_i) - group_trajectory_(i - 1, position_i) < -position_limit[position_i])
				{
					amount = group_trajectory_(i - 1, position_i) - position_limit[position_i] - group_trajectory_(i, position_i);
					absolute_amount = fabs(amount);
				}
				if (absolute_amount > max_abs_violation)
				{
					max_abs_violation = absolute_amount;
					max_violation = amount;
					max_violation_index = i;
					violation = true;
				}
			}

			if (violation)
			{
				ROS_DEBUG_NAMED("chomp_optimizer", "violation handle");
				int free_var_index = max_violation_index - free_vars_start_;
				double multiplier = max_violation /
														position_costs_->getQuadraticCostInverse()(
																free_var_index, free_var_index);
				group_trajectory_.getFreePositionTrajectoryBlock(position_i) +=
						multiplier *
						position_costs_->getQuadraticCostInverse().col(free_var_index);
			}
			if (++count > 10)
				break;
		} while (violation);
	}
}


void ChompOptimizer::handleLimits()
{
	for (int i = free_vars_start_ + 1; i <= free_vars_end_; i++)
	{
		if (group_trajectory_(i, 2) - group_trajectory_(i - 1, 2) > max_theta_)
		{
			group_trajectory_(i, 2) = group_trajectory_(i - 1, 2) + max_theta_;
		}
		else if (group_trajectory_(i, 2) - group_trajectory_(i - 1, 2) < -max_theta_)
		{
			group_trajectory_(i, 2) = group_trajectory_(i - 1, 2) - max_theta_;
		}

		double discretization = full_trajectory_->getDiscretization();

		double delta_x = group_trajectory_(i, 0) - group_trajectory_(i - 1, 0);
		double delta_y = group_trajectory_(i, 1) - group_trajectory_(i - 1, 1);
		double vel = sqrt((delta_x * delta_x + delta_y * delta_y) / (discretization * discretization));
		double vel_x = vel * cos(group_trajectory_(i, 2));
		double vel_y = vel * sin(group_trajectory_(i, 2));
		double max_x = vel_x * discretization;
		double max_y = vel_y * discretization;

		if(group_trajectory_(i, 0) - group_trajectory_(i - 1, 0) > max_x)
		{
			group_trajectory_(i, 0) = group_trajectory_(i - 1, 0) + max_x;
		}
		else if(group_trajectory_(i, 0) - group_trajectory_(i - 1, 0) < -max_x)
		{
			group_trajectory_(i, 0) = group_trajectory_(i - 1, 0) - max_x;
		}

		if(group_trajectory_(i, 1) - group_trajectory_(i - 1, 1) > max_x)
		{
			group_trajectory_(i, 1) = group_trajectory_(i - 1, 1) + max_x;
		}
		else if(group_trajectory_(i, 1) - group_trajectory_(i - 1, 1) < -max_x)
		{
			group_trajectory_(i, 1) = group_trajectory_(i - 1, 1) - max_x;
		}
	}
}