#include <base_local_planner/goal_functions.h>
#include <chomp_local_planner/chomp_planner.h>
#include <cmath>

// for computing path distance
#include <queue>

#include <angles/angles.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2/utils.h>
#include <chomp_local_planner/chomp_car_trajectory.h>
#include <tf/tf.h>
#include <chomp_local_planner/chomp_utils.h>

#include <chomp_local_planner/chomp_optimizer.h>


namespace chomp_trajectory
{
ChompTrajectory::ChompTrajectory(double timeval, unsigned int point_num, std::vector<geometry_msgs::PoseStamped> &input_path, double linVel, double angVel, double angle)
	:	input_path_(input_path), 
		timeval_(timeval),
		point_num_(point_num)
{
	output_path_.resize(point_num_);
	//delta_t_ = timeval / static_cast<double>(point_num - 1);
  //delta_t_ = timeval;
  delta_t_ = 0.3;
	output_path_[0] = Vector2d(linVel, angVel);
	angleAll.resize(point_num_);
	angleAll[0] = angle;

}

vector<Vector2d> ChompTrajectory::getTrajectory()
{
	for(unsigned int i = 1; i < point_num_; i++)
	{
    double delta_x = input_path_[i].pose.position.x - input_path_[i-1].pose.position.x;
    double delta_y = input_path_[i].pose.position.y - input_path_[i-1].pose.position.y;

		double linVel = sqrt( (delta_x*delta_x + delta_y*delta_y) / (delta_t_*delta_t_));

		double angle = atan2(delta_y, delta_x);

		angleAll[i] = angle;

		double angVel = (angle - angleAll[i-1]) / delta_t_;

		output_path_[i] = Vector2d(linVel, angVel);

   // ROS_INFO("output_path: %f, %f", output_path_[i][0], output_path_[i][1]);
	}	

	return output_path_;
}
}

namespace chomp_local_planner
{
CHOMPPlanner::CHOMPPlanner(std::string name, base_local_planner::LocalPlannerUtil* planner_util)
  : planner_util_(planner_util)
{
  ros::NodeHandle private_nh("~/" + name);

  std::string controller_frequency_param_name;
  if (!private_nh.searchParam("controller_frequency", controller_frequency_param_name))
  {
    sim_period_ = 0.05;
  }
  else
  {
    double controller_frequency = 0;
    private_nh.param(controller_frequency_param_name, controller_frequency, 20.0);
    if (controller_frequency > 0)
    {
      sim_period_ = 1.0 / controller_frequency;
    }
    else
    {
      ROS_WARN("A controller_frequency less than 0 has been set. Ignoring the parameter, assuming a rate of 20Hz");
      sim_period_ = 0.05;
    }
  }
  ROS_INFO("Sim period is set to %.2f", sim_period_);

  preGlobalPath_pub = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
}

bool CHOMPPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
{
  return planner_util_->setPlan(orig_global_plan);
}

void CHOMPPlanner::updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
                                           const std::vector<geometry_msgs::PoseStamped>& new_plan,
                                           const std::vector<geometry_msgs::Point>& footprint_spec)
{
  global_plan_.resize(new_plan.size());
  for (unsigned int i = 0; i < new_plan.size(); ++i)
  {
    global_plan_[i] = new_plan[i]; 
  }
  
}


void CHOMPPlanner::reconfigure(CHOMPPlannerConfig &config)
{
  boost::mutex::scoped_lock l(configuration_mutex_);

  ROS_DEBUG_NAMED("chomp_planner", "CHOMPPlanner reconfigure");
}
/*
bool CHOMPPlanner::checkTrajectory(
      Eigen::Vector3f pos,
      Eigen::Vector3f vel,
      Eigen::Vector3f vel_samples){
    oscillation_costs_.resetOscillationFlags();
    base_local_planner::Trajectory traj;
    geometry_msgs::PoseStamped goal_pose = global_plan_.back();
    Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf2::getYaw(goal_pose.pose.orientation));
    base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
    generator_.initialise(pos,
        vel,
        goal,
        &limits,
        vsamples_);
    generator_.generateTrajectory(pos, vel, vel_samples, traj);
    double cost = scored_sampling_planner_.scoreTrajectory(traj, -1);
    //if the trajectory is a legal one... the check passes
    if(cost >= 0) {
      return true;
    }
    ROS_WARN("Invalid Trajectory %f, %f, %f, cost: %f", vel_samples[0], vel_samples[1], vel_samples[2], cost);

    //otherwise the check fails
    return false;
  }*/

std::vector<geometry_msgs::PoseStamped> CHOMPPlanner::findBestPath(
          const geometry_msgs::PoseStamped& global_pose, 
          std::string baseFrameID, 
          const geometry_msgs::PoseStamped& global_vel, 
          std::vector<Vector2d>& drive_velocities, double yaw_initial)
{
  ROS_DEBUG_NAMED("chomp_planner", "CHOMPPlanner findBestPath");
  //chomp_trajectory::ChompTrajectory trajectory(sim_period_, global_plan_.size(), global_plan_, 0, 0, yaw_initial);
  
  //drive_velocities = trajectory.getTrajectory();


/*
  vector<double> linVel;
  vector<double> angVel;
  for (int i = 0; i < drive_velocities.size(); i++)
  {
    linVel.push_back(drive_velocities[i][0]);
    angVel.push_back(drive_velocities[i][1]);
  }

  double x = 0.0;
  double y = 0.0;
  double th = yaw_initial;
  nav_msgs::Path outPath;
  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();
  outPath.header.stamp = current_time;
  outPath.header.frame_id = "odom";

  for (int i = 0; i < drive_velocities.size(); i++)
  {
    current_time = ros::Time::now();

   // double dt = (current_time - last_time).toSec();
    //double dt = sim_period_;
    double dt = 0.3;
    double delta_x = drive_velocities[i][0] * cos(th) * dt;
    double delta_y = drive_velocities[i][0] * sin(th) * dt;
    double delta_th = drive_velocities[i][1] * dt;

    ROS_INFO("vel: %f, %f", drive_velocities[i][0], drive_velocities[i][1]);

    x += delta_x;
    y += delta_y;
    th += delta_th;

    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = x;
    this_pose_stamped.pose.position.y = y;

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(th);
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp = current_time;
    this_pose_stamped.header.frame_id = "odom";

    outPath.poses.push_back(this_pose_stamped);
    preGlobalPath_pub.publish(outPath);
    last_time = current_time;
    ros::Duration(0.05);
  }*/
  
  ChompCarTrajectory::ChompTrajectory trajectory(global_plan_, yaw_initial, sim_period_);

  //std::cout<<"original trajectory:"<<std::endl;
  //std::cout<<trajectory.getTrajectory()<<std::endl;
  /*
  std::cout<<"start index: "<<trajectory.getStartIndex()<<endl;
  std::cout<<"end index: "<<trajectory.getEndIndex()<<endl;
  std::cout<<"num points: "<<trajectory.getNumPoints()<<endl;
  cout<<"free num points: "<<trajectory.getNumFreePoints()<<endl;
  cout<<"get position trajectroy: "<<trajectory.getPositionTrajectory(0)<<endl;
  cout<<"---------------------"<<endl;

  ChompCarTrajectory::ChompTrajectory group_trajectory(trajectory, DIFF_RULE_LENGTH);
  std::cout<<"group_trajectory: "<<group_trajectory.getTrajectory()<<endl;
  std::cout<<"start index: "<<group_trajectory.getStartIndex()<<endl;
  std::cout<<"end index: "<<group_trajectory.getEndIndex()<<endl;
  std::cout<<"num points: "<<group_trajectory.getNumPoints()<<endl;
  cout<<"free num points: "<<group_trajectory.getNumFreePoints()<<endl;
  cout<<"get position trajectroy: "<<group_trajectory.getPositionTrajectory(0)<<endl;
  cout<<"get free trajectory block: "<<group_trajectory.getFreeTrajectoryBlock()<<endl;
  cout<<"get free joint trajectory block: "<<group_trajectory.getFreePositionTrajectoryBlock(1)<<endl;*/


  ChompOptimizer chompOptimizer(&trajectory);
  chompOptimizer.optimize();

  Eigen::MatrixXd local_trajectory = trajectory.getTrajectory();
  for(int i = 0; i < trajectory.getNumPoints(); ++i)
  {
    global_plan_[i].pose.position.x = local_trajectory(i, 0);
    global_plan_[i].pose.position.y = local_trajectory(i, 1);
  }

  return global_plan_;
}


}  // namespace chomp_local_planner


