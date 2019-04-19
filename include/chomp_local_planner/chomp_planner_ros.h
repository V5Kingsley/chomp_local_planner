#ifndef CHOMP_LOCAL_PLANNER_CHOMP_PLANNER_ROS_H_
#define CHOMP_LOCAL_PLANNER_CHOMP_PLANNER_ROS_H_

#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf2_ros/buffer.h>

#include <chomp_local_planner/CHOMPPlannerConfig.h>
#include <dwa_local_planner/DWAPlannerConfig.h>
#include <dynamic_reconfigure/server.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <base_local_planner/latched_stop_rotate_controller.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>

#include <base_local_planner/odometry_helper_ros.h>

#include <chomp_local_planner/chomp_planner.h>

#include <vector>

namespace chomp_local_planner
{
class CHOMPPlannerROS : public nav_core::BaseLocalPlanner
{
public:
  CHOMPPlannerROS();

  void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros);

  ~CHOMPPlannerROS()
  {
    delete dsrv_;
  };

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  bool chompComputeVelocityCommands(geometry_msgs::PoseStamped &global_pose, geometry_msgs::Twist &cmd_vel);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);

  bool isInitialized()
  {
    return initialized_;
  }

  bool isGoalReached();

private:
  bool initialized_;

  // for visualisation, publishers of global and local plan
  ros::Publisher g_plan_pub_, l_plan_pub_;

  tf2_ros::Buffer *tf_;  ///< @brief Used for transforming point clouds

  costmap_2d::Costmap2DROS *costmap_ros_;

  geometry_msgs::PoseStamped current_pose_;

  base_local_planner::LocalPlannerUtil planner_util_;  //用来存储运动控制参数以及costmap2d、tf等，会被传入dp_. by
                                                       //Kingsley

  dynamic_reconfigure::Server<CHOMPPlannerConfig> *dsrv_;
  chomp_local_planner::CHOMPPlannerConfig default_config_;
  bool setup_;

  base_local_planner::OdometryHelperRos odom_helper_;

  std::string odom_topic_;

  boost::shared_ptr<CHOMPPlanner> cp_;

  base_local_planner::LatchedStopRotateController latchedStopRotateController_;

  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped> &path);

  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped> &path);

  void reconfigureCB(CHOMPPlannerConfig &config, uint32_t level);

  ros::Publisher vel_pub_;
  ros::NodeHandle nh;

  void costmap_test();


};
};  // namespace chomp_local_planner

#endif  //  CHOMP_LOCAL_PLANNER_CHOMP_PLANNER_ROS_H_