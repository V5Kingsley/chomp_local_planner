#ifndef CHOMP_LOCAL_PALNNER_CHOMP_LOCAL_PLANNER_H_
#define CHOMP_LOCAL_PALNNER_CHOMP_LOCAL_PLANNER_H_

#include <chomp_local_planner/CHOMPPlannerConfig.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

#include <Eigen/Core>
#include <vector>

// for creating a local cost grid
#include <base_local_planner/map_grid_visualizer.h>

// for obstacle data access
#include <costmap_2d/costmap_2d.h>

#include <base_local_planner/local_planner_limits.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/simple_trajectory_generator.h>
#include <base_local_planner/trajectory.h>

#include <base_local_planner/map_grid_cost_function.h>
#include <base_local_planner/obstacle_cost_function.h>
#include <base_local_planner/oscillation_cost_function.h>
#include <base_local_planner/simple_scored_sampling_planner.h>
#include <base_local_planner/twirling_cost_function.h>

#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#include <base_local_planner/odometry_helper_ros.h>
using namespace std;
using namespace Eigen;

#include "tf/transform_datatypes.h"

namespace chomp_local_planner
{
class CHOMPPlanner
{
public:
  CHOMPPlanner(std::string name, base_local_planner::LocalPlannerUtil* planner_util);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);

  double getSimPeriod()
  {
    return sim_period_;
  }

  void updatePlanAndLocalCosts(const geometry_msgs::PoseStamped& global_pose,
                               const std::vector<geometry_msgs::PoseStamped>& new_plan,
                               const std::vector<geometry_msgs::Point>& footprint_spec);

  void reconfigure(CHOMPPlannerConfig& config);

  std::vector<geometry_msgs::PoseStamped> findBestPath(const geometry_msgs::PoseStamped& global_pose,
                                                       std::string baseFrameID,
                                                       const geometry_msgs::PoseStamped& global_vel,
                                                       std::vector<Vector2d>& drive_velocities, base_local_planner::OdometryHelperRos *odom_helper);

  bool checkTrajectory(Eigen::Vector3f pos, Eigen::Vector3f vel, Eigen::Vector3f vel_samples);

private:
  base_local_planner::LocalPlannerUtil* planner_util_;

  std::vector<geometry_msgs::PoseStamped> global_plan_;

  double sim_period_;

  boost::mutex configuration_mutex_;

 


  ros::NodeHandle nh;
  ros::Publisher preGlobalPath_pub;

  std::vector<geometry_msgs::Point> footprint_;

  void getCmdVel(Eigen::MatrixXd &position_trajectory, base_local_planner::OdometryHelperRos *odom_helper, std::vector<Vector2d>& drive_velocities);

  base_local_planner::SimpleTrajectoryGenerator generator_;
  base_local_planner::SimpleScoredSamplingPlanner scored_sampling_planner_;
  base_local_planner::OscillationCostFunction oscillation_costs_;
  base_local_planner::ObstacleCostFunction obstacle_costs_;
  base_local_planner::MapGridCostFunction path_costs_;
  base_local_planner::MapGridCostFunction goal_costs_;
  base_local_planner::MapGridCostFunction goal_front_costs_;
  base_local_planner::MapGridCostFunction alignment_costs_;
  base_local_planner::TwirlingCostFunction twirling_costs_;
  double pdist_scale_, gdist_scale_, occdist_scale_;
  Eigen::Vector3f vsamples_;
  double stop_time_buffer_;
  double forward_point_distance_;
  double cheat_factor_;
};

}  // namespace chomp_local_planner


#endif  //  CHOMP_LOCAL_PALNNER_CHOMP_LOCAL_PLANNER_H_