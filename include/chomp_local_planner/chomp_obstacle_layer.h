/**
 * @file chomp_obstacle_layer.h
 * @author Kingsley
 * @brief chomp 障碍物层，包含每个点的自身坐标信息，距离最近的障碍物距离以及对应障碍物点的坐标信息
 * @version 0.1
 * @date 2019-04-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */

#ifndef CHOMP_OBSTACLE_LAYER_H_
#define CHOMP_OBSTACLE_LAYER_H_

#include <costmap_2d/costmap_2d.h>
#include <eigen3/Eigen/Core>
#include <chomp_local_planner/chomp_parameters.h>

namespace chomp_obstacle_layer
{
class CellData
{
public:
  /**
   * @brief  Constructor for a CellData objects
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @param  the distance between(x, y) and (sx, xy)
   * @return
   */
  CellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy, double distance) :
      index_(i), x_(x), y_(y), src_x_(sx), src_y_(sy), clear_(false)
  {
  }

  //默认构造函数，设置clear_为true
  CellData() : clear_(true), index_(-1), x_(-1), y_(-1), src_x_(-1), src_y_(-1), distance_(-1)
  {
  }

  /**
   * @brief Set the Cell Data object
   * 
   * @param  i The index of the cell in the cost map
   * @param  x The x coordinate of the cell in the cost map
   * @param  y The y coordinate of the cell in the cost map
   * @param  sx The x coordinate of the closest obstacle cell in the costmap
   * @param  sy The y coordinate of the closest obstacle cell in the costmap
   * @param  the distance between(x, y) and (sx, xy)
   * 
   */
  void setCellData(double i, unsigned int x, unsigned int y, unsigned int sx, unsigned int sy, double distance)
  {
    index_ = i;
    x_ = x;
    y_ = y;
    src_x_ = sx;
    src_y_ = sy;
    distance_ = distance;
    clear_ = false;
  }
  unsigned int index_;
  unsigned int x_, y_;
  unsigned int src_x_, src_y_;
  double distance_;
  bool clear_;   //clear_: true means the cell if clear of obstacles; false means the cell is near obstacles. 
};


class ChompObstacleLayer
{
public:
  ChompObstacleLayer(costmap_2d::Costmap2D *costmap, double obstacle_dis);

  bool getObstacleDist(double wx, double wy, double &obs_x, double &obs_y, double &distance);

  void viewCostMap();

  void viewObstacleCells();

  void getPotential(std::vector<double> &state, double &potential, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> *potential_gradient, int index, double &distance);

  inline double calculatePotential(double distance);

private:
  costmap_2d::Costmap2D *map_grid_;
  unsigned int cell_inflation_radius_;  //距离障碍物的最远考虑距离（栅格地图距离）
  double inflation_radius_;  //距离障碍物的最远考虑距离（世界距离）

  std::map<double, std::vector<CellData> > inflation_cells_;

  std::vector<CellData> obstacle_cells_; //存储障碍物层每个栅格点信息

  unsigned char** cached_costs_;
  double** cached_distances_;

  bool* seen_;

//从世界距离转换为栅格图距离
  unsigned int cellDistance(double world_dist)
  {
    return map_grid_->cellDistance(world_dist);
  }

  void computeCashes();
  void computeObstacleCells();

//从cached_distances中获取距离
  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }

  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y);

  std::vector<double> robot_vertex_[num_collision_points];  //机器人四个顶点坐标（相对于机体坐标系）
};




} // namespace chomp_obstacle_layer

#endif  // CHOMP_OBSTACLE_LAYER_H_