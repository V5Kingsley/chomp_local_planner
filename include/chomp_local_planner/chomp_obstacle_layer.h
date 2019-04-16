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
  CellData() : clear_(true), index_(-1), x_(-1), y_(-1), src_x_(-1), src_y_(-1), distance_(-1)
  {
  }

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
  bool clear_;
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
  unsigned int cell_inflation_radius_;
  double inflation_radius_;

  std::map<double, std::vector<CellData> > inflation_cells_;

  std::vector<CellData> obstacle_cells_;

  unsigned char** cached_costs_;
  double** cached_distances_;

  bool* seen_;

  unsigned int cellDistance(double world_dist)
  {
    return map_grid_->cellDistance(world_dist);
  }

  void computeCashes();
  void computeObstacleCells();

  inline double distanceLookup(int mx, int my, int src_x, int src_y)
  {
    unsigned int dx = abs(mx - src_x);
    unsigned int dy = abs(my - src_y);
    return cached_distances_[dx][dy];
  }

  inline void enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y);

  std::vector<double> robot_vertex_[num_collision_points];
};




} // namespace chomp_obstacle_layer

#endif  // CHOMP_OBSTACLE_LAYER_H_