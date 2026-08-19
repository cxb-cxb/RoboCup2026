#ifndef MISSION_NODE_MISSION_PARAMETERS_H
#define MISSION_NODE_MISSION_PARAMETERS_H

#include <ros/ros.h>

struct MissionParameters {
  double fx = 1092.34009;
  double fy = 1088.58832;
  double cx = 657.880369;
  double cy = 361.681183;
  double dx = 0.0;
  double dy = 0.21;
  double dz = 0.0;
  double max_velocity_x = 0.5;
  double max_velocity_y = 0.5;
  double drop_height = 1.0;
  double gravity = 9.8;
  double land_velocity = 0.2;
  double history_duration = 5.0;
  double search_x_min = -4.0;
  double search_x_max = 4.0;
  double search_y_min = -4.0;
  double search_y_max = 4.0;
  double search_height = 1.0;
  double search_footprint_x = 2.0;
  double search_footprint_y = 1.5;
  double search_overlap_ratio = 0.25;
  int max_search_passes = 3;

  void load(const ros::NodeHandle& private_nh);
  void validate();
  void log() const;
  double dropTime() const;
};

#endif
