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
  map_node.param("_resolution", grid_resolution, 0.1);
  map_node.param("_map_schema", map_schema, false);

  _idx_max = ceil(_global_x_size / grid_resolution);
  _idy_max = ceil(_global_y_size / grid_resolution);
  _idz_max = ceil(_global_z_size / grid_resolution);

  total_index = _idx_max * _idy_max * _idz_max;

  // global_data = std::vector<double>(_idx_max * _idy_max * _idz_max, 0.0);
  occ_state_data = std::vector<short>(_idx_max * _idy_max * _idz_max, 0.0);

  update_local_timestamp = ros::Time().fromSec(0.0);

  local_occ_timer = nh.createTimer(ros::Duration(0.5),
                                   &GridMap::updateLocalOccCallback, this);
}

unsigned GridMap::coordToBufferIndex(Eigen::Vector3d coord) {
  return ceil(coord(0) / grid_resolution) * ceil(coord(1) / grid_resolution) *
         ceil(coord(2) / grid_resolution);
}

unsigned GridMap::posToBufferIndex(Eigen::Vector3i pos) {
  return pos(2) * (_idx_max * _idy_max) + pos(1) * _idx_max + pos(0);
}

Eigen::Vector3i GridMap::bufferIndexToPos(unsigned int index) {
  Eigen::Vector3i pos;
  if (index > total_index) out_range = true;
  int z = ceil(index / (_idx_max * _idy_max));
  int y = ceil((index - z * _idx_max * _idy_max) / _idx_max);
  int x = index - z * (_idx_max * _idy_max) - y * _idy_max;
  pos << x, y, z;
  return pos;
}

void GridMap::computeEsdf(Eigen::Vector3d pos) {}

void GridMap::updateLocalOccCallback(const ros::TimerEvent &e) {
  if (update_local_timestamp.toSec() < 1.0) {
    update_local_occ_flag = true;
    update_local_timestamp = ros::Time::now();
  }

  // TODO: consider odom timeout.
  rayTracing()
}

}  // namespace plan_frame
