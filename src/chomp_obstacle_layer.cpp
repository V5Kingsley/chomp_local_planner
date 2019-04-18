/**
 * @file chomp_obstacle_layer.cpp
 * @author Kingsley
 * @brief 
 * @version 0.1
 * @date 2019-04-16
 * 
 * @copyright Copyright (c) 2019
 * 
 */
#include <chomp_local_planner/chomp_obstacle_layer.h>

#include <costmap_2d/cost_values.h>

#include <ros/ros.h>

namespace chomp_obstacle_layer
{
/**
 * @brief Construct a new Chomp Obstacle Layer:: Chomp Obstacle Layer object
 * 
 * @param costmap 
 * @param obstacle_dis 距离障碍物的最远考虑距离，小于该距离需计算potential，否则potential为0
 */
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

/**
 * @brief 预先计算好cell_inflation_radius_内的距离，可节省计算步骤
 * @ 如cached_distances_[i][j]表示两个x轴相距i，y轴相距j的栅格距离
 * 
 */
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
/**
 * @brief 计算障碍物层信息
 * 
 */
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

/**
 * @brief 获取输入点对应的障碍物信息
 * 
 * @param wx 输入世界坐标x
 * @param wy 输入世界坐标y
 * @param obs_x 输出该坐标距离最近的障碍物坐标x
 * @param obs_y 输出该坐标距离最近的障碍物坐标y
 * @param distance 输出距离最近障碍物的距离
 * @return true 该点在障碍物附近，返回相应的距离
 * @return false 该点距离障碍物足够远，返回距离为-1
 */
bool ChompObstacleLayer::getObstacleDist(double wx, double wy, double &obs_x, double &obs_y, double &distance)
{
  unsigned int mx, my;
  map_grid_->worldToMap(wx, wy, mx, my);
  unsigned int index = map_grid_->getIndex(mx, my);

  //map_grid_->setCost(mx, my, 100);

  if(index >= obstacle_cells_.size())
  {
    distance = -1;
    return false;
    viewCostMap();
    ROS_ERROR("(%f, %f) (%f, %f) (%d, %d), index is out of map", map_grid_->getOriginX(), map_grid_->getOriginY(),wx, wy, mx, my);
    
  }
    
  
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

/**
 * @brief 获取势能，势能梯度，距障碍物的最小距离
 * 
 * @param state 输入状态量(x, y, theta)
 * @param potential 输出势能
 * @param potential_gradient 输出势能梯度 
 * @param index 输入轨迹点的index
 * @param distance 输出距障碍物的最小距离
 */
void ChompObstacleLayer::getPotential(std::vector<double> &state, double &potential, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> *potential_gradient, int index, double &distance)
{

  double min_distance = 1000;
  distance = min_distance;
  potential = 0; //重置为0

  double wx = state[0];
  double wy = state[1];
  double obs_x, obs_y;

  getObstacleDist(wx, wy, obs_x, obs_y, min_distance); //获取障碍物信息

  if(min_distance != -1)
      distance = min_distance < distance ? min_distance : distance;

  //分别计算机器人的四个顶点
  for (int i = 0; i < num_collision_points; i++)
  {

    //clear时，势能梯度为0
    if (min_distance == -1)
    {
      (*(potential_gradient + i))[index](0, 0) = (*(potential_gradient + i))[index](1, 0) = (*(potential_gradient + i))[index](2, 0) = 0;
      //ROS_DEBUG_NAMED("chomp_obstacle_layer", "This trajectory point is clear");
      continue;
    }

    //当顶点在障碍物内时，将势能梯度设为0（由于障碍物内的距离都设为了0,没有设为负，无法计算梯度）
    if(min_distance == 0)
    {
      (*(potential_gradient + i))[index](0, 0) = (*(potential_gradient + i))[index](1, 0) = (*(potential_gradient + i))[index](2, 0) = 0;
      potential += calculatePotential(min_distance);
      //ROS_DEBUG_NAMED("chomp_obstacle_layer", "This trajectory point is in obstacle, potential: %f", potential);
      continue;
    }

//以下为计算势能和势能梯度（在障碍物附近时）

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
                   (-robot_vertex_[i][0] * sin(state[2]) - 
                   robot_vertex_[i][1] * cos(state[2]));

    double temp2 = (robot_vertex_[i][0] * sin(state[2]) +
                    robot_vertex_[i][1] * cos(state[2]) + state[1] - obs_y) *
                   (robot_vertex_[i][0] * cos(state[2]) - 
                   robot_vertex_[i][1] * sin(state[2]));
    
    (*(potential_gradient + i))[index](2, 0) = (temp1 + temp2) / min_distance;

   // ROS_DEBUG_NAMED("chomp_obstacle_layer","calculate gradient over");
  }
  double vertex_distance;
  getObstacleDist(state[0] + 0.1, state[1], obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
  getObstacleDist(state[0] - 0.1, state[1], obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
  getObstacleDist(state[0], state[1] + 0.1, obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
  getObstacleDist(state[0], state[1] - 0.1, obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
}

/**
 * @brief 计算势能,对应论文p4左上角
 * 
 * @param distance 输入距离障碍物的距离
 * @return double 返回势能
 */
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


/*void ChompObstacleLayer::getPotential(std::vector<double> &state, double &potential, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> *potential_gradient, int index, double &distance)
{
  double min_distance = 1000;
  distance = min_distance;
  potential = 0;

  double robot_x = state[0];
  double robot_y = state[1];
  double obs_x, obs_y;
  getObstacleDist(robot_x, robot_y, obs_x, obs_y, distance);
  
  for(int i = 0; i < num_collision_points; i++)
  {
    //clear时，势能梯度为0
    if(distance == -1)
    {
      (*(potential_gradient + i))[index](0, 0) = (*(potential_gradient + i))[index](1, 0) = (*(potential_gradient + i))[index](2, 0) = 0;
      //ROS_DEBUG_NAMED("chomp_obstacle_layer", "This trajectory point is clear");
      continue;
    }

    //当顶点在障碍物内时，将势能梯度设为0（由于障碍物内的距离都设为了0,没有设为负，无法计算梯度）
    if(distance == 0)
    {
      (*(potential_gradient + i))[index](0, 0) = (*(potential_gradient + i))[index](1, 0) = (*(potential_gradient + i))[index](2, 0) = 0;
      potential += calculatePotential(distance);
      //ROS_DEBUG_NAMED("chomp_obstacle_layer", "This trajectory point is in obstacle, potential: %f", potential);
      continue;
    }

    //以下为计算势能和势能梯度（在障碍物附近时）

    potential += calculatePotential(distance);

    //ROS_DEBUG_NAMED("chomp_obstacle_layer", "vertex: (%f, %f), obstacle: (%f, %f), distance: %f, potential: %f", wx, wy, obs_x, obs_y, min_distance, potential);

    (*(potential_gradient + i))[index](0, 0) =
        (robot_vertex_[i][0] * cos(state[2]) -
         robot_vertex_[i][1] * sin(state[2]) + state[0] - obs_x) /
        distance;
    
    (*(potential_gradient + i))[index](1, 0) =
        (robot_vertex_[i][0] * sin(state[2]) +
         robot_vertex_[i][1] * cos(state[2]) + state[1] - obs_y) /
        distance;

    double temp1 = (robot_vertex_[i][0] * cos(state[2]) -
                    robot_vertex_[i][1] * sin(state[2]) + state[0] - obs_x) *
                   (-robot_vertex_[i][0] * sin(state[2]) - 
                   robot_vertex_[i][1] * cos(state[2]));

    double temp2 = (robot_vertex_[i][0] * sin(state[2]) +
                    robot_vertex_[i][1] * cos(state[2]) + state[1] - obs_y) *
                   (robot_vertex_[i][0] * cos(state[2]) - 
                   robot_vertex_[i][1] * sin(state[2]));
    
    (*(potential_gradient + i))[index](2, 0) = (temp1 + temp2) / distance;
  }

  //计算机体中心周围的四个点是否碰到障碍物
  double vertex_distance;
  getObstacleDist(robot_x + 0.05, robot_y, obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
  getObstacleDist(robot_x - 0.05, robot_y, obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
  getObstacleDist(robot_x, robot_y + 0.05, obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
  getObstacleDist(robot_x, robot_y - 0.05, obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
}*/


/*void ChompObstacleLayer::getPotential(std::vector<double> &state, double &potential, std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> *potential_gradient, int index, double &distance)
{

  double min_distance = 1000;
  distance = min_distance;
  potential = 0; //重置为0

  //ROS_DEBUG_NAMED("chomp_obstacle_layer", "(%f, %f), (%f, %f), (%f, %f), (%f, %f)", robot_vertex_[0][0], robot_vertex_[0][1], robot_vertex_[1][0], robot_vertex_[1][1], robot_vertex_[2][0], robot_vertex_[2][1], robot_vertex_[3][0], robot_vertex_[3][1]);

  //分别计算机器人的四个顶点
  for(int i = 0; i < num_collision_points; i++)
  {
    //顶点的世界坐标
   // double wx = robot_vertex_[i][0] * cos(state[2]) - robot_vertex_[i][1] * sin(state[2]) + state[0];
    //double wy = robot_vertex_[i][0] * sin(state[2]) + robot_vertex_[i][1] * cos(state[2]) + state[1];

    double wx = state[0];
    double wy = state[1];
    //if(i == 0)
    //{
    //  wx = state[0];
    //  wy = state[1];
    //}
   // ROS_DEBUG_NAMED("chomp_obstacle_layer", "theta: %f", state[2]);

    double obs_x, obs_y;

    getObstacleDist(wx, wy, obs_x, obs_y, min_distance);  //获取障碍物信息
    //ROS_DEBUG_NAMED("chomp_obstacle_layer", "vertex: (%f, %f), obstacle: (%f, %f), distance: %f", wx, wy, obs_x, obs_y, min_distance);

    if(min_distance != -1)
      distance = min_distance < distance ? min_distance : distance;

    //clear时，势能梯度为0
    if(min_distance == -1)
    {
      (*(potential_gradient + i))[index](0, 0) = (*(potential_gradient + i))[index](1, 0) = (*(potential_gradient + i))[index](2, 0) = 0;
      //ROS_DEBUG_NAMED("chomp_obstacle_layer", "This trajectory point is clear");
      continue;
    }

    //当顶点在障碍物内时，将势能梯度设为0（由于障碍物内的距离都设为了0,没有设为负，无法计算梯度）
    if(min_distance == 0)
    {
      (*(potential_gradient + i))[index](0, 0) = (*(potential_gradient + i))[index](1, 0) = (*(potential_gradient + i))[index](2, 0) = 0;
      potential += calculatePotential(min_distance);
      //ROS_DEBUG_NAMED("chomp_obstacle_layer", "This trajectory point is in obstacle, potential: %f", potential);
      continue;
    }

//以下为计算势能和势能梯度（在障碍物附近时）

    potential = calculatePotential(min_distance);

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
                   (-robot_vertex_[i][0] * sin(state[2]) - 
                   robot_vertex_[i][1] * cos(state[2]));

    double temp2 = (robot_vertex_[i][0] * sin(state[2]) +
                    robot_vertex_[i][1] * cos(state[2]) + state[1] - obs_y) *
                   (robot_vertex_[i][0] * cos(state[2]) - 
                   robot_vertex_[i][1] * sin(state[2]));
    
    (*(potential_gradient + i))[index](2, 0) = (temp1 + temp2) / min_distance;

   // ROS_DEBUG_NAMED("chomp_obstacle_layer","calculate gradient over");
  }
  double vertex_distance, obs_x, obs_y;
  getObstacleDist(state[0] + 0.1, state[1], obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
  getObstacleDist(state[0] - 0.1, state[1], obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
  getObstacleDist(state[0], state[1] + 0.1, obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
  getObstacleDist(state[0], state[1] - 0.1, obs_x, obs_y, vertex_distance);
  if(vertex_distance == 0)
    distance = 0;
}*/
}  // namespace chomp_obstacle_layer