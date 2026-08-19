#include "CoverageSearchPlanner.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace {
std::vector<double> makeScanLines(double minimum, double maximum,
                                  double footprint, double step) {
  std::vector<double> lines;
  const double first = minimum + footprint * 0.5;
  const double last = maximum - footprint * 0.5;

  if (first > last) {
    lines.push_back((minimum + maximum) * 0.5);
    return lines;
  }

  for (double value = first; value <= last + 1e-6; value += step) {
    lines.push_back(std::min(value, last));
  }
  if (lines.empty() || std::abs(lines.back() - last) > 1e-6) {
    lines.push_back(last);
  }
  return lines;
}
}  // namespace

CoverageSearchPlanner::CoverageSearchPlanner()
    : x_min_(-4.0),
      x_max_(4.0),
      y_min_(-4.0),
      y_max_(4.0),
      height_(1.0),
      footprint_x_(2.0),
      footprint_y_(1.5),
      overlap_ratio_(0.25),
      max_passes_(3),
      pass_count_(0),
      current_index_(0) {}

void CoverageSearchPlanner::configure(double x_min, double x_max, double y_min,
                                      double y_max, double height,
                                      double footprint_x, double footprint_y,
                                      double overlap_ratio, int max_passes) {
  x_min_ = x_min;
  x_max_ = x_max;
  y_min_ = y_min;
  y_max_ = y_max;
  height_ = height;
  footprint_x_ = footprint_x;
  footprint_y_ = footprint_y;
  overlap_ratio_ = overlap_ratio;
  max_passes_ = max_passes;
}

void CoverageSearchPlanner::generate() {
  waypoints_.clear();
  visited_.clear();
  current_index_ = 0;
  pass_count_ = 1;

  const double width = x_max_ - x_min_;
  const double height = y_max_ - y_min_;
  const double x_step = footprint_x_ * (1.0 - overlap_ratio_);
  const double y_step = footprint_y_ * (1.0 - overlap_ratio_);

  if (width >= height) {
    const std::vector<double> rows =
        makeScanLines(y_min_, y_max_, footprint_y_, y_step);
    const double left = (x_min_ + footprint_x_ * 0.5 <=
                         x_max_ - footprint_x_ * 0.5)
                            ? x_min_ + footprint_x_ * 0.5
                            : (x_min_ + x_max_) * 0.5;
    const double right = (x_min_ + footprint_x_ * 0.5 <=
                          x_max_ - footprint_x_ * 0.5)
                             ? x_max_ - footprint_x_ * 0.5
                             : left;
    for (std::size_t row = 0; row < rows.size(); ++row) {
      if (row % 2 == 0) {
        addWaypoint(left, rows[row]);
        if (right != left) addWaypoint(right, rows[row]);
      } else {
        addWaypoint(right, rows[row]);
        if (right != left) addWaypoint(left, rows[row]);
      }
    }
  } else {
    const std::vector<double> columns =
        makeScanLines(x_min_, x_max_, footprint_x_, x_step);
    const double bottom = (y_min_ + footprint_y_ * 0.5 <=
                           y_max_ - footprint_y_ * 0.5)
                              ? y_min_ + footprint_y_ * 0.5
                              : (y_min_ + y_max_) * 0.5;
    const double top = (y_min_ + footprint_y_ * 0.5 <=
                        y_max_ - footprint_y_ * 0.5)
                           ? y_max_ - footprint_y_ * 0.5
                           : bottom;
    for (std::size_t column = 0; column < columns.size(); ++column) {
      if (column % 2 == 0) {
        addWaypoint(columns[column], bottom);
        if (top != bottom) addWaypoint(columns[column], top);
      } else {
        addWaypoint(columns[column], top);
        if (top != bottom) addWaypoint(columns[column], bottom);
      }
    }
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
