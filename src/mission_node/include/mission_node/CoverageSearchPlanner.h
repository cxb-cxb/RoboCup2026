#ifndef MISSION_NODE_COVERAGE_SEARCH_PLANNER_H
#define MISSION_NODE_COVERAGE_SEARCH_PLANNER_H

#include <cstddef>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>

class CoverageSearchPlanner {
 public:
  CoverageSearchPlanner();

  void configure(const std::vector<double>& points_x,
                 const std::vector<double>& points_y,
                 double height, int max_passes);
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

  std::vector<double> points_x_;
  std::vector<double> points_y_;
  double height_;
  int max_passes_;
  int pass_count_;
  std::size_t current_index_;
  std::vector<geometry_msgs::PoseStamped> waypoints_;
  std::vector<bool> visited_;
};

#endif
