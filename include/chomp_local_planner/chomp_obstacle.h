/**
 * @file chomp_obstacle.h
 * @author Kingsley 
 * @brief 
 * @version 0.1
 * @date 2019-04-10
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#ifndef CHOMP_OBSTACLE_H_
#define CHOMP_OBSTACLE_H_

#include <costmap_2d/costmap_2d.h>

namespace chomp_obstacle
{
static const int MAX_OBSTACLE_COST = 200;

enum STATUS
{
  CLEAR,
  IN_OBSTACLE,
  FOUND,
  NOT_FOUND
};

class ChompObstacle
{
public:
  ChompObstacle(costmap_2d::Costmap2D* costmap, double clear_dist);

  ~ChompObstacle()
  {
  }

  STATUS getMinDistanceAndCoordinate(double wx, double wy, double& distance, double& obstacle_wx, double& obstacle_wy);

  void viewCostMap();

private:
  costmap_2d::Costmap2D* costmap_;
  double clear_dist_;
  unsigned int obstacle_size_x_;
  unsigned int obstacle_size_y_;
  double resolution_;
  unsigned int obstacle_clear_dist_;

  std::vector<geometry_msgs::Point> foot_print_;

  double calculateInObstacle(double mx, double my, double wx, double wy, double& obstacle_wx, double& obstacle_wy);

};

}  // namespace chomp_obstacle

#endif
