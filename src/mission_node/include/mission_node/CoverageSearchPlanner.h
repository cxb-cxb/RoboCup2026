#ifndef MISSION_NODE_COVERAGE_SEARCH_PLANNER_H
#define MISSION_NODE_COVERAGE_SEARCH_PLANNER_H

#include <cstddef>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

class CoverageSearchPlanner {
 public:
  CoverageSearchPlanner();

  void configure(double x_min, double x_max, double y_min, double y_max,
                 double height, double footprint_x, double footprint_y,
                 double overlap_ratio, int max_passes);
  void generate();

  bool empty() const;
  bool hasCurrentWaypoint() const;
  const geometry_msgs::PoseStamped& currentWaypoint() const;
  void markCurrentVisited();
  bool resumeNearest(const geometry_msgs::Point& current_position);
  bool startNextPass(const geometry_msgs::Point& current_position);
  int passCount() const;
  bool exhausted() const;

 private:
  void addWaypoint(double x, double y);
  std::size_t nearestUnvisited(const geometry_msgs::Point& current_position) const;

  double x_min_;
  double x_max_;
  double y_min_;
  double y_max_;
  double height_;
  double footprint_x_;
  double footprint_y_;
  double overlap_ratio_;
  int max_passes_;
  int pass_count_;
  std::size_t current_index_;
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  std::vector<bool> visited_;
};

#endif
