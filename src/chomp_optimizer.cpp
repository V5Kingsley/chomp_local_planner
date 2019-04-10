#include <chomp_local_planner/chomp_optimizer.h>

ChompOptimizer::ChompOptimizer(ChompTrajectory *trajectory)
	:	full_trajectory_(trajectory),
		group_trajectory_(*trajectory, DIFF_RULE_LENGTH)
{
	num_vars_free_ = group_trajectory_.getNumFreePoints();
  num_vars_all_ = group_trajectory_.getNumPoints();

	free_vars_start_ = group_trajectory_.getStartIndex();
  free_vars_end_ = group_trajectory_.getEndIndex();

	std::vector<double> derivative_costs(3);
	derivative_costs[0] = ChompParameters::getSmoothnessCostVelocity();
	derivative_costs[1] = ChompParameters::getSmoothnessCostAcceleration();
	derivative_costs[2] = ChompParameters::getSmoothnessCostJerk();
	position_costs_.reset(new StompCost(group_trajectory_, derivative_costs, 
																			ChompParameters::getRidgeFactor()));

	double cost_scale = position_costs_->getMaxQuadCostInvValue();
	double max_cost_scale = 0.0;
	if (max_cost_scale < cost_scale)
    max_cost_scale = cost_scale;
  //  放缩joint_costs_矩阵，类似于SHOMP文章第三页算法中Precompute的第三行
  position_costs_->scale(max_cost_scale);

	// 对矩阵分配内存
  smoothness_increments_ =
      Eigen::MatrixXd::Zero(num_vars_free_, NUM_POSITION_PLAN);
  smoothness_derivative_ = Eigen::VectorXd::Zero(num_vars_all_);

}

bool ChompOptimizer::optimize()
{
	original_group_trajectory_ = group_trajectory_.getTrajectory();
	best_group_trajectory_ = group_trajectory_.getTrajectory();
	double best_group_trajectory_cost = 10000;

	int iteration;
	int max_iteration = ChompParameters::getMaxIterations();
	for(iteration = 0; iteration < max_iteration; ++iteration)
	{
		ROS_DEBUG_NAMED("chomp_optimizer", "iteration[%d]", iteration);

		double smoothCost = getSmoothnessCost();
		double collisionCost = 0;
		double cost = smoothCost + collisionCost;

		ROS_DEBUG_NAMED("chomp_optimizer", "cost: %f", cost);
		if (cost < best_group_trajectory_cost) 
		{
      best_group_trajectory_ = group_trajectory_.getTrajectory();
      best_group_trajectory_cost = cost;
    }

		calculateSmoothnessIncrements();

    addIncrementsToTrajectory();
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
	}

	group_trajectory_.getTrajectory() = best_group_trajectory_;
	updateFullTrajectory();

	std::cout<<"After iteration, the trajectory becomes: "<<std::endl;
	std::cout<<full_trajectory_->getTrajectory();
}

double ChompOptimizer::getSmoothnessCost()
{
	double smoothness_cost = 0.0;
	for(unsigned int i = 0; i < NUM_POSITION_PLAN; ++i)
	{
		smoothness_cost +=
				position_costs_->getCost(group_trajectory_.getPositionTrajectory(i));
	}
	return ChompParameters::getSmoothnessCostWeight() * smoothness_cost;
}

void ChompOptimizer::calculateSmoothnessIncrements()
{
	for(unsigned int i = 0; i < NUM_POSITION_PLAN; ++i)
	{
		position_costs_->getDerivative(group_trajectory_.getPositionTrajectory(i),
																	smoothness_derivative_);
		smoothness_increments_.col(i) = -smoothness_derivative_.segment(
        group_trajectory_.getStartIndex(), num_vars_free_);  //segment(i,n)获取第i个元素开始的n个元素
	}
}

void ChompOptimizer::addIncrementsToTrajectory()
{
	Eigen::MatrixXd final_increments;
  final_increments = Eigen::MatrixXd::Zero(num_vars_free_, NUM_POSITION_PLAN);

	for(unsigned i = 0; i < NUM_POSITION_PLAN; ++i)
	{
		final_increments.col(i) = 
			ChompParameters::getLearningRate() *
			(position_costs_->getQuadraticCostInverse() * 
				(ChompParameters::getSmoothnessCostWeight() * smoothness_increments_.col(i))
			);
		double scale = 1.0;
		group_trajectory_.getFreeTrajectoryBlock().col(i) += 
				scale * final_increments.col(i);
	}

}


void ChompOptimizer::updateFullTrajectory() 
{
  full_trajectory_->updateFromGroupTrajectory(group_trajectory_);
}
