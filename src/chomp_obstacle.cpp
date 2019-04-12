/**
 * @file chomp_obstacle.cpp
 * @author Kingsley 
 * @brief 
 * @version 0.1
 * @date 2019-04-10
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#include <chomp_local_planner/chomp_obstacle.h>
#include <ros/ros.h>
#include <iostream>
#include <cmath>

#define DISTANCE_BETWEEN_POINT(x1, y1, x2, y2) sqrt( (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) ) 


namespace chomp_obstacle
{
ChompObstacle::ChompObstacle(costmap_2d::Costmap2D* costmap, double clear_dist)
  : clear_dist_(clear_dist)
  , costmap_(costmap)
  , resolution_(costmap->getResolution())
  , obstacle_size_x_(costmap->getSizeInCellsX())
  , obstacle_size_y_(costmap->getSizeInCellsY())
  , obstacle_clear_dist_(clear_dist / resolution_)
{
}

/**
 * @brief 获取距离输入点最近的障碍物坐标以及距离信息,x,y,distance
 * 
 * @param wx   input world coordination x
 * @param wy   input world coordination y
 * @param distance   output distance to nearest obstacle
 * @param obstacle_wx   output nearest obstacle wrold coordination
 * @param obstacle_wy   output nearest obstacle wrold coordination
 * @return STATUS 
 */
STATUS ChompObstacle::getMinDistanceAndCoordinate(double wx, double wy, double& distance, double& obstacle_wx,
                                                  double& obstacle_wy)
{
  unsigned int mx;
  unsigned int my;

  if (costmap_->worldToMap(wx, wy, mx, my))
  {
    ROS_DEBUG_NAMED("chomp_obstacle", "wx: %f, wy: %f, mx: %d, my: %d", wx, wy, mx, my);
    //costmap_->setCost(mx, my, 1);
    if(costmap_->getCost(mx, my) > MAX_OBSTACLE_COST)
    {
      distance = calculateInObstacle(mx, my, wx, wy, obstacle_wx, obstacle_wy);
      ROS_DEBUG_NAMED("chomp_obstacle", "This trajectory point is in obstacle, distance: %f", distance);
      return IN_OBSTACLE;
    }
    for (int i = 1; i <= obstacle_clear_dist_; ++i)
    {
      if (mx - i > obstacle_size_x_)
      {
        for (int range_y = my - i; range_y <= my + i; ++range_y)
        {
          if (range_y < 0 || range_y > obstacle_size_y_)
            break;
          if (costmap_->getCost(mx - i, range_y) > MAX_OBSTACLE_COST)
          {
            costmap_->mapToWorld(mx - i, range_y, obstacle_wx, obstacle_wy);
            distance = DISTANCE_BETWEEN_POINT(wx, wy, obstacle_wx, obstacle_wy);
            costmap_->setCost(mx - i, range_y, 200);
            ROS_DEBUG_NAMED("chomp_obstacle", "the nearest obstacle coodination: (%f, %f), distance: %f", obstacle_wx, obstacle_wy, distance);
            return FOUND;
          }
        }
      }

      if (my + i < obstacle_size_y_)
      {
        for (int range_x = mx - i; range_x <= mx + i; ++range_x)
        {
          if (range_x < 0 || range_x > obstacle_size_x_)
            break;
          if (costmap_->getCost(range_x, my + i) > MAX_OBSTACLE_COST)
          {
            costmap_->mapToWorld(range_x, my + i, obstacle_wx, obstacle_wy);
            distance = DISTANCE_BETWEEN_POINT(wx, wy, obstacle_wx, obstacle_wy);
            costmap_->setCost(range_x, my + i, 200);
            ROS_DEBUG_NAMED("chomp_obstacle", "the nearest obstacle coodination: (%f, %f), distance: %f", obstacle_wx, obstacle_wy, distance);
            return FOUND;
          }
        }
      }

      if (mx + i < obstacle_size_x_)
      {
        for (int range_y = my - i; range_y <= my + i; ++range_y)
        {
          if (range_y < 0 || range_y > obstacle_size_y_)
            break;
          if (costmap_->getCost(mx + i, range_y) > MAX_OBSTACLE_COST)
          {
            costmap_->mapToWorld(mx + i, range_y, obstacle_wx, obstacle_wy);
            distance = DISTANCE_BETWEEN_POINT(wx, wy, obstacle_wx, obstacle_wy);
            costmap_->setCost(mx + i, range_y, 200);
            ROS_DEBUG_NAMED("chomp_obstacle", "the nearest obstacle coodination: (%f, %f), distance: %f", obstacle_wx, obstacle_wy, distance);
            return FOUND;
          }
        }
      }

      if (my - i > 0)
      {
        for (int range_x = mx - i; range_x <= mx + i; ++range_x)
        {
          if (range_x < 0 || range_x > obstacle_size_x_)
            break;
          if (costmap_->getCost(range_x, my - i) > MAX_OBSTACLE_COST)
          {
            costmap_->mapToWorld(range_x, my - i, obstacle_wx, obstacle_wy);
            distance = DISTANCE_BETWEEN_POINT(wx, wy, obstacle_wx, obstacle_wy);
            costmap_->setCost(range_x, my - i, 200);
            ROS_DEBUG_NAMED("chomp_obstacle", "the nearest obstacle coodination: (%f, %f), distance: %f", obstacle_wx, obstacle_wy, distance);
            return FOUND;
          }
        }
      }
    }
  }
  else
  {
    ROS_WARN_NAMED("chomp_obstacle", "get error, wx: %f, wy: %f, origin_x: %f, origin_y: %f", wx, wy,
                   costmap_->getOriginX(), costmap_->getOriginY());
    ROS_WARN_NAMED("chomp_obstacle", "got mx: %d, my: %d", static_cast<int>(mx), static_cast<int>(my));
    return NOT_FOUND;
  }

  ROS_DEBUG_NAMED("chomp_obstacle", "This trajectory point(%f, %f) is clear", wx, wy);
  return CLEAR;
}

void ChompObstacle::viewCostMap()
{
  int size_x = costmap_->getSizeInCellsX();
  int size_y = costmap_->getSizeInCellsY();
  // ROS_DEBUG_NAMED("costmap_test", "size_x: %d, size_y: %d", size_x, size_y);
  std::cout << "costmap: " << size_x << " * " << size_y << std::endl;
  std::cout << "costmap resolution: " << costmap_->getResolution() << std::endl;
  for (int i = 0; i < size_x; ++i)
  {
    for (int j = 0; j < size_y; ++j)
    {
      std::cout << (unsigned int)costmap_->getCost(i, j) << " " << std::setw(4);
    }
    std::cout << std::endl;
  }
}

double ChompObstacle::calculateInObstacle(double mx, double my, double wx, double wy, double& obstacle_wx, double& obstacle_wy)
{
  for(int i = 0; i < obstacle_size_x_; ++i)
  {
    if(mx - i > obstacle_size_x_)
    {
      for(int range_y = my - i; range_y <= my + i; ++range_y)
      {
        if (range_y < 0 || range_y > obstacle_size_y_)
          break;
        if (costmap_->getCost(mx - i, range_y) < MAX_OBSTACLE_COST)
        {
          costmap_->mapToWorld(mx - i, range_y, obstacle_wx, obstacle_wy);
          return -DISTANCE_BETWEEN_POINT(wx, wy, obstacle_wx, obstacle_wy);
        }
      }
    }

    if (my + i < obstacle_size_y_)
      {
        for (int range_x = mx - i; range_x <= mx + i; ++range_x)
        {
          if (range_x < 0 || range_x > obstacle_size_x_)
            break;
          if (costmap_->getCost(range_x, my + i) < MAX_OBSTACLE_COST)
          {
            costmap_->mapToWorld(range_x, my + i, obstacle_wx, obstacle_wy);
            return -DISTANCE_BETWEEN_POINT(wx, wy, obstacle_wx, obstacle_wy);
            
          }
        }
      }


    if (mx + i < obstacle_size_x_)
      {
        for (int range_y = my - i; range_y <= my + i; ++range_y)
        {
          if (range_y < 0 || range_y > obstacle_size_y_)
            break;
          if (costmap_->getCost(mx + i, range_y) < MAX_OBSTACLE_COST)
          {
            costmap_->mapToWorld(mx + i, range_y, obstacle_wx, obstacle_wy);
            return -DISTANCE_BETWEEN_POINT(wx, wy, obstacle_wx, obstacle_wy);
          }
        }
      }

      if (my - i > 0)
      {
        for (int range_x = mx - i; range_x <= mx + i; ++range_x)
        {
          if (range_x < 0 || range_x > obstacle_size_x_)
            break;
          if (costmap_->getCost(range_x, my - i) < MAX_OBSTACLE_COST)
          {
            costmap_->mapToWorld(range_x, my - i, obstacle_wx, obstacle_wy);
            return -DISTANCE_BETWEEN_POINT(wx, wy, obstacle_wx, obstacle_wy);
          }
        }
      }
  }

  return -100000;

  ROS_WARN_NAMED("chomp_obstacle", "This local map is all in collision!!!");
}


}  // namespace chomp_obstacle