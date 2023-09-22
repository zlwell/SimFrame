//
// Created by zl on 23-9-21.
//

#include "grid_map/grid_map.h"

namespace plan_frame {

void GridMap::initMap(ros::NodeHandle &nh) {
  map_node = nh;

  map_node.param("_global_x_size", _global_x_size, 50.0);
  map_node.param("_global_y_size", _global_y_size, 50.0);
  map_node.param("_global_z_size", _global_z_size, 5.0);

  _idx_max = ceil(_global_x_size / grid_resolution);
  _idy_max = ceil(_global_y_size / grid_resolution);
  _idz_max = ceil(_global_z_size / grid_resolution);

  total_index = _idx_max * _idy_max * _idz_max;

  global_data = std::vector<double>(_idx_max * _idy_max * _idz_max, 0.0);
  occ_state_data = std::vector<short>(_idx_max * _idy_max * _idz_max, 0.0);

  // TODO: local map data init.
}

unsigned GridMap::posToIndex(Eigen::Vector3d position) {
  return ceil(position(0) / grid_resolution) *
         ceil(position(1) / grid_resolution) *
         ceil(position(2) / grid_resolution);
}

Eigen::Vector3d GridMap::indexToPos(unsigned index) {
  Eigen::Vector3d pos;
  if (index > total_index) out_range = true;
  int z = floor(index / (_idx_max * _idy_max));
  int y = floor((index - z * _idx_max * _idy_max)/_idx_max)

}

}  // namespace plan_frame
