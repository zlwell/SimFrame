//
// Created by zl on 23-9-21.
//

#ifndef GRID_MAP_H_
#define GRID_MAP_H_

#include <Eigen/Eigen>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <cmath>

namespace plan_frame {

class GridMap {
private:

  ros::NodeHandle map_node;

  double _global_x_size, _global_y_size, _global_z_size;
  double _local_x_size, _local_y_size, _local_z_size;

  double grid_resolution;

  unsigned _idx_max, _idy_max, _idz_max;
  unsigned long total_index;

  std::vector<double> global_data;
  std::vector<short> occ_state_data;
  std::vector<double> local_data;

  Eigen::Vector3d origin;

  bool map_schema;
  bool out_range;

public:
  GridMap(){};
  ~GridMap(){};

  void initMap(ros::NodeHandle &nh);

  unsigned posToIndex(Eigen::Vector3d postion);
  Eigen::Vector3d indexToPos(unsigned index);

  void publishMapPCL(ros::NodeHandle &nh);
};

} // namespace plan_frame

#endif // GRID_MAP_H_
