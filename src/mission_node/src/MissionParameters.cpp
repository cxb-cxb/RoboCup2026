#include "MissionParameters.h"

namespace {
void restoreDefaultSearchPoints(std::vector<double>& points_x,
                                std::vector<double>& points_y) {
  points_x = {-3.0, 3.0, 3.0, -3.0, -3.0, 3.0};
  points_y = {-3.0, -3.0, 0.0, 0.0, 3.0, 3.0};
}

template <typename T>
void restoreIfNotPositive(T& value, const T& default_value,
                          const char* parameter_name) {
  if (value > 0) {
    return;
  }

  ROS_WARN_STREAM("Parameter '~" << parameter_name << "' must be positive; using "
                                  << default_value);
  value = default_value;
}
}  // namespace

void MissionParameters::load(const ros::NodeHandle& private_nh) {
  private_nh.param("camera/fx", fx, fx);
  private_nh.param("camera/fy", fy, fy);
  private_nh.param("camera/cx", cx, cx);
  private_nh.param("camera/cy", cy, cy);
  private_nh.param("camera/offset_x", dx, dx);
  private_nh.param("camera/offset_y", dy, dy);
  private_nh.param("camera/offset_z", dz, dz);
  private_nh.param("control/max_velocity_x", max_velocity_x, max_velocity_x);
  private_nh.param("control/max_velocity_y", max_velocity_y, max_velocity_y);
  private_nh.param("drop/height", drop_height, drop_height);
  private_nh.param("drop/gravity", gravity, gravity);
  private_nh.param("drop/land_velocity", land_velocity, land_velocity);
  private_nh.param("filter/history_duration", history_duration, history_duration);
  private_nh.param("search/height", search_height, search_height);
  private_nh.param("search/max_passes", max_search_passes, max_search_passes);
  const bool has_points_x = private_nh.getParam("search/points_x", search_points_x);
  const bool has_points_y = private_nh.getParam("search/points_y", search_points_y);
  if (!has_points_x || !has_points_y) {
    ROS_WARN("Search feature-point arrays are incomplete; using default path");
    restoreDefaultSearchPoints(search_points_x, search_points_y);
  }

  validate();
  log();
}

void MissionParameters::validate() {
  restoreIfNotPositive(fx, 1092.34009, "camera/fx");
  restoreIfNotPositive(fy, 1088.58832, "camera/fy");
  restoreIfNotPositive(max_velocity_x, 0.5, "control/max_velocity_x");
  restoreIfNotPositive(max_velocity_y, 0.5, "control/max_velocity_y");
  restoreIfNotPositive(drop_height, 1.0, "drop/height");
  restoreIfNotPositive(gravity, 9.8, "drop/gravity");
  restoreIfNotPositive(land_velocity, 0.2, "drop/land_velocity");
  restoreIfNotPositive(history_duration, 5.0, "filter/history_duration");
  restoreIfNotPositive(search_height, 1.0, "search/height");
  if (search_points_x.empty() || search_points_y.empty() ||
      search_points_x.size() != search_points_y.size()) {
    ROS_WARN("Invalid search feature-point arrays; using default path");
    restoreDefaultSearchPoints(search_points_x, search_points_y);
  }
  restoreIfNotPositive(max_search_passes, 3, "search/max_passes");
}

double MissionParameters::dropTime() const {
  return drop_height / land_velocity;
}

void MissionParameters::log() const {
  ROS_INFO_STREAM("Mission parameters loaded: camera(fx=" << fx << ", fy=" << fy
                  << ", cx=" << cx << ", cy=" << cy << ", offset=[" << dx
                  << ", " << dy << ", " << dz << "]), max_velocity=["
                  << max_velocity_x << ", " << max_velocity_y << "], drop(height="
                  << drop_height << ", gravity=" << gravity << ", land_velocity="
                  << land_velocity << ", time=" << dropTime()
                  << "), history_duration=" << history_duration
                  << ", search(points=" << search_points_x.size()
                  << ", height=" << search_height << ", max_passes="
                  << max_search_passes << ")");
}
