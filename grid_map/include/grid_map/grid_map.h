//
// Created by zl on 23-9-21.
//

#ifndef GRID_MAP_H_
#define GRID_MAP_H_

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <ros/time.h>

#include <Eigen/Eigen>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace plan_frame {

class GridMap {
 private:
  ros::NodeHandle map_node;

  double _global_x_size, _global_y_size, _global_z_size;
  double _local_x_size, _local_y_size, _local_z_size;

  double grid_resolution;

  unsigned _idx_max, _idy_max, _idz_max;
  unsigned long total_index;

  std::vector<short> occ_state_data;  // OCC data buffer.
  std::vector<double> distance_data;  // ESDF data buffer.

  Eigen::Vector3d origin;

  bool map_schema;  // true : real flight; false: simulation
  bool out_range;
  bool update_local_occ_flag;
  ros::Time update_local_timestamp;

  ros::Timer local_occ_timer;



 public:
  GridMap(){};
  ~GridMap(){};

  void initMap(ros::NodeHandle &nh);

  // convert coordinates to buffer index.
  unsigned coordToBufferIndex(Eigen::Vector3d coord);

  // convert <x, y, z> index to an integer index in the buffer.
  unsigned posToBufferIndex(Eigen::Vector3i position);

  // convert buffer index to <x, y ,z> index format.
  Eigen::Vector3i bufferIndexToPos(unsigned index);

  void publishMapPCL(ros::NodeHandle &nh);

  // TODO: compute esdf
  void computeEsdf(Eigen::Vector3d pos);

  void updateLocalOccCallback(const ros::TimerEvent& e)
};

}  // namespace plan_frame

#endif  // GRID_MAP_H_
