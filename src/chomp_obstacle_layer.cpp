#include <chomp_local_planner/chomp_obstacle_layer.h>

#include <costmap_2d/cost_values.h>

#include <ros/ros.h>

namespace chomp_obstacle_layer
{
ChompObstacleLayer::ChompObstacleLayer(costmap_2d::Costmap2D *costmap, double obstacle_dis) 
  : map_grid_(costmap), inflation_radius_(obstacle_dis)
{
  cell_inflation_radius_ = cellDistance(obstacle_dis);

  for(int i = 0; i < num_collision_points; ++i)
  {
    robot_vertex_[i].resize(2);
    RobotParameter::getRobotVertex(i, robot_vertex_[i][0], robot_vertex_[i][1]);
  }
  computeCashes();

  computeObstacleCells();
}

void ChompObstacleLayer::computeCashes()
{
  ROS_DEBUG_NAMED("chomp_obstacle_layer", "computeCashes.");
  if (cell_inflation_radius_ == 0)
    return;

  //cached_costs_ = new unsigned char *[cell_inflation_radius_ + 2];
  cached_distances_ = new double *[cell_inflation_radius_ + 2];

  for (unsigned int i = 0; i <= cell_inflation_radius_ + 1; ++i)
  {
    //cached_costs_[i] = new unsigned char[cell_inflation_radius_ + 2];
    cached_distances_[i] = new double[cell_inflation_radius_ + 2];
    for (unsigned int j = 0; j <= cell_inflation_radius_ + 1; ++j)
    {
      cached_distances_[i][j] = hypot(i, j);
    }
  }
}

void ChompObstacleLayer::computeObstacleCells()
{
  ROS_DEBUG_NAMED("chomp_obstacle_layer", "computeObstacleCells.");
  unsigned char* grid_cost_array = map_grid_->getCharMap();
  unsigned int size_x = map_grid_->getSizeInCellsX();
  unsigned int size_y = map_grid_->getSizeInCellsY();

  seen_ = new bool[size_x * size_y];
  memset(seen_, false, size_x * size_y * sizeof(bool));

  obstacle_cells_.resize(size_x * size_y);

  // Start with lethal obstacles: by definition distance is 0.0
  ROS_DEBUG_NAMED("chomp_obstacle_layer", "push back lethal_obstacle.");
  std::vector<CellData>& obs_bin = inflation_cells_[0.0];
  for(int j = 0; j < size_y; ++j)
  {
    for(int i = 0; i < size_x; ++i)
    {
      int index = map_grid_->getIndex(i, j);
      unsigned char cost  = grid_cost_array[index];
      if(cost > costmap_2d::LETHAL_OBSTACLE - 2)
      {
        obs_bin.push_back(CellData(index, i, j, i, j, 0));
        obstacle_cells_[index].setCellData(index, i, j, i, j, 0);
      }
    }
  }

  // Process cells by increasing distance; new cells are appended to the corresponding distance bin, so they
  // can overtake previously inserted but farther away cells
  ROS_DEBUG_NAMED("chomp_obstacle_layer", "push back inflation.");
  std::map<double, std::vector<CellData> >::iterator bin;
  for (bin = inflation_cells_.begin(); bin != inflation_cells_.end(); ++bin)
  {
    for(int i = 0; i < bin->second.size(); ++i)
    {
      const CellData& cell = bin->second[i];
      unsigned int index = cell.index_;
      
      if (seen_[index])
      {
        continue;
      }

      seen_[index] = true;

      unsigned int mx = cell.x_;
      unsigned int my = cell.y_;
      unsigned int sx = cell.src_x_;
      unsigned int sy = cell.src_y_;

      // attempt to put the neighbors of the current cell onto the inflation list
      if (mx > 0)
        enqueue(index - 1, mx - 1, my, sx, sy);
      if (my > 0)
        enqueue(index - size_x, mx, my - 1, sx, sy);
      if (mx < size_x - 1)
        enqueue(index + 1, mx + 1, my, sx, sy);
      if (my < size_y - 1)
        enqueue(index + size_x, mx, my + 1, sx, sy);
    }
  }

}


inline void ChompObstacleLayer::enqueue(unsigned int index, unsigned int mx, unsigned int my,
                      unsigned int src_x, unsigned int src_y)
{
  if(!seen_[index])
  {
    unsigned char* grid_cost_array = map_grid_->getCharMap();
    if(grid_cost_array[index] == costmap_2d::LETHAL_OBSTACLE)
      return;

    double distance = distanceLookup(mx, my, src_x, src_y);

    if(distance > cell_inflation_radius_)
      return;
    
    inflation_cells_[distance].push_back(CellData(index, mx, my, src_x, src_y, distance));
    obstacle_cells_[index].setCellData(index, mx, my, src_x, src_y, distance);
  }
}

bool ChompObstacleLayer::getObstacleDist(double wx, double wy, double &obs_x, double &obs_y, double &distance)
{
  unsigned int mx, my;
  map_grid_->worldToMap(wx, wy, mx, my);
  unsigned int index = map_grid_->getIndex(mx, my);

  if(obstacle_cells_[index].clear_)
  {
    distance = -1;
    //ROS_DEBUG_NAMED("chomp_obstacle_layer", "this point is clear of obstacles");
    return false;
  }
  else
  {
    map_grid_->mapToWorld(obstacle_cells_[index].src_x_, obstacle_cells_[index].src_y_, obs_x, obs_y);
    double cell_distance = obstacle_cells_[index].distance_;
    distance = cell_distance * map_grid_->getResolution();
    return true;
  }
  
}


void ChompObstacleLayer::viewCostMap()
{
  int size_x = map_grid_->getSizeInCellsX();
  int size_y = map_grid_->getSizeInCellsY();
  // ROS_DEBUG_NAMED("costmap_test", "size_x: %d, size_y: %d", size_x, size_y);
  std::cout << "costmap: " << size_x << " * " << size_y << std::endl;
  std::cout << "costmap resolution: " << map_grid_->getResolution() << std::endl;
  for (int i = 0; i < size_x; ++i)
  {
    for (int j = 0; j < size_y; ++j)
    {
      std::cout << (unsigned int)map_grid_->getCost(i, j) << " " << std::setw(4);
    }
    std::cout << std::endl;
  }
}

void ChompObstacleLayer::viewObstacleCells()
{
  for(int i = 0; i < obstacle_cells_.size(); ++i)
  {
    if(i % 60 == 0)
      std::cout<<std::endl;

    std::cout<<obstacle_cells_[i].distance_<<" "<<std::setw(4)<<std::setprecision(3);
  }
}

void ChompObstacleLayer::getPotential(std::vector<double> &state, double &potential, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> *potential_gradient, int index, double &distance)
{

  double min_distance = 1000;
  distance = min_distance;
  potential = 0;
  for(int i = 0; i < num_collision_points; i++)
  {
    double wx = robot_vertex_[i][0] * cos(state[2]) - robot_vertex_[i][1] * sin(state[2]) + state[0];
    double wy = robot_vertex_[i][0] * sin(state[2]) + robot_vertex_[i][1] * cos(state[2]) + state[1];
    //double wx = state[0];
    //double wy = state[1];
    double obs_x, obs_y;

    getObstacleDist(wx, wy, obs_x, obs_y, min_distance);
    //ROS_DEBUG_NAMED("chomp_obstacle_layer", "vertex: (%f, %f), obstacle: (%f, %f), distance: %f", wx, wy, obs_x, obs_y, min_distance);

    if(min_distance != -1)
      distance = min_distance < distance ? min_distance : distance;

    // calculate potential gradient
    if(min_distance == -1)
    {
      (*(potential_gradient + i))[index](0, 0) = (*(potential_gradient + i))[index](1, 0) = (*(potential_gradient + i))[index](2, 0) = 0;
      //ROS_DEBUG_NAMED("chomp_obstacle_layer", "This trajectory point is clear");
      continue;
    }

    if(min_distance == 0)
    {
      (*(potential_gradient + i))[index](0, 0) = (*(potential_gradient + i))[index](1, 0) = (*(potential_gradient + i))[index](2, 0) = 0;
      potential += calculatePotential(min_distance);
      //ROS_DEBUG_NAMED("chomp_obstacle_layer", "This trajectory point is in obstacle, potential: %f", potential);
      continue;
    }

    potential += calculatePotential(min_distance);

    //ROS_DEBUG_NAMED("chomp_obstacle_layer", "vertex: (%f, %f), obstacle: (%f, %f), distance: %f, potential: %f", wx, wy, obs_x, obs_y, min_distance, potential);

    (*(potential_gradient + i))[index](0, 0) =
        (robot_vertex_[i][0] * cos(state[2]) -
         robot_vertex_[i][1] * sin(state[2]) + state[0] - obs_x) /
        min_distance;
    
    (*(potential_gradient + i))[index](1, 0) =
        (robot_vertex_[i][0] * sin(state[2]) +
         robot_vertex_[i][1] * cos(state[2]) + state[1] - obs_y) /
        min_distance;

    double temp1 = (robot_vertex_[i][0] * cos(state[2]) -
                    robot_vertex_[i][1] * sin(state[2]) + state[0] - obs_x) *
                   (-robot_vertex_[i][0] * sin(state[2] - 
                   robot_vertex_[i][1] * cos(state[2])));

    double temp2 = (robot_vertex_[i][0] * sin(state[2]) +
                    robot_vertex_[i][1] * cos(state[2]) + state[1] - obs_y) *
                   (robot_vertex_[i][0] * cos(state[2] - 
                   robot_vertex_[i][1] * sin(state[2])));
    
    (*(potential_gradient + i))[index](2, 0) = (temp1 + temp2) / min_distance;

   // ROS_DEBUG_NAMED("chomp_obstacle_layer","calculate gradient over");
  }
}

inline double ChompObstacleLayer::calculatePotential(double distance)
{
  if(distance > inflation_radius_ || distance == -1)
    return 0;
  else
  {
    double potential;
    double temp = (distance - inflation_radius_) * (distance - inflation_radius_);
    potential = 1.0 / 2.0 * inflation_radius_ * temp;
    return potential;
  }
}

}  // namespace chomp_obstacle_layer