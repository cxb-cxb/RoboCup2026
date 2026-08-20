#include "CoverageSearchPlanner.h"

#include <algorithm>
#include <limits>

CoverageSearchPlanner::CoverageSearchPlanner()
    : points_x_({-3.0, 3.0, 3.0, -3.0, -3.0, 3.0}),
      points_y_({-3.0, -3.0, 0.0, 0.0, 3.0, 3.0}),
      height_(1.0),
      max_passes_(3),
      pass_count_(0),
      current_index_(0) {}

void CoverageSearchPlanner::configure(const std::vector<double>& points_x,
                                      const std::vector<double>& points_y,
                                      double height, int max_passes) {
  points_x_ = points_x;
  points_y_ = points_y;
  height_ = height;
  max_passes_ = max_passes;
}

void CoverageSearchPlanner::generate() {
  waypoints_.clear();
  visited_.clear();
  current_index_ = 0;
  pass_count_ = 1;

  const std::size_t count = std::min(points_x_.size(), points_y_.size());
  for (std::size_t i = 0; i < count; ++i) {
    addWaypoint(points_x_[i], points_y_[i]);
  }

  visited_.assign(waypoints_.size(), false);
}

bool CoverageSearchPlanner::empty() const { return waypoints_.empty(); }

bool CoverageSearchPlanner::hasCurrentWaypoint() const {
  return current_index_ < waypoints_.size() && !visited_[current_index_];
}

const geometry_msgs::PoseStamped& CoverageSearchPlanner::currentWaypoint() const {
  return waypoints_.at(current_index_);
}

void CoverageSearchPlanner::markCurrentVisited() {
  if (current_index_ >= visited_.size()) return;
  visited_[current_index_] = true;
  while (current_index_ < visited_.size() && visited_[current_index_]) {
    ++current_index_;
  }
}

bool CoverageSearchPlanner::resumeNearest(
    const geometry_msgs::Point& current_position) {
  const std::size_t nearest = nearestUnvisited(current_position);
  if (nearest >= waypoints_.size()) return false;
  current_index_ = nearest;
  return true;
}

bool CoverageSearchPlanner::startNextPass(
    const geometry_msgs::Point& current_position) {
  if (exhausted() || waypoints_.empty()) return false;
  ++pass_count_;
  std::reverse(waypoints_.begin(), waypoints_.end());
  visited_.assign(waypoints_.size(), false);
  return resumeNearest(current_position);
}

int CoverageSearchPlanner::passCount() const { return pass_count_; }

bool CoverageSearchPlanner::exhausted() const {
  return pass_count_ >= max_passes_;
}

void CoverageSearchPlanner::addWaypoint(double x, double y) {
  geometry_msgs::PoseStamped waypoint;
  waypoint.header.frame_id = "camera_init";
  waypoint.pose.position.x = x;
  waypoint.pose.position.y = y;
  waypoint.pose.position.z = height_;
  waypoint.pose.orientation.w = 1.0;
  waypoints_.push_back(waypoint);
}

std::size_t CoverageSearchPlanner::nearestUnvisited(
    const geometry_msgs::Point& current_position) const {
  std::size_t nearest = waypoints_.size();
  double nearest_squared_distance = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < waypoints_.size(); ++i) {
    if (visited_[i]) continue;
    const double dx = waypoints_[i].pose.position.x - current_position.x;
    const double dy = waypoints_[i].pose.position.y - current_position.y;
    const double squared_distance = dx * dx + dy * dy;
    if (squared_distance < nearest_squared_distance) {
      nearest_squared_distance = squared_distance;
      nearest = i;
    }
  }
  return nearest;
}
