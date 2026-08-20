# Feature Point Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace automatically generated coverage rows with six parameterized feature points connected by direct EGO-Planner flight segments.

**Architecture:** `MissionParameters` loads paired X/Y coordinate arrays. `CoverageSearchPlanner` retains its current navigation interface but builds its waypoint list directly from those arrays, allowing the existing `MissionFSM` search, interruption, resume, priority, and drop logic to remain unchanged.

**Tech Stack:** ROS 1 roscpp, C++11, geometry_msgs, catkin, YAML

**Verification constraint:** Per user instruction, do not install dependencies or run compilation, tests, simulation, or hardware verification.

---

### Task 1: Replace coverage geometry parameters with feature-point arrays

**Files:**
- Modify: `src/mission_node/include/mission_node/MissionParameters.h`
- Modify: `src/mission_node/src/MissionParameters.cpp`
- Modify: `src/mission_node/config/mission_params.yaml`

- [ ] Remove `search_x_min`, `search_x_max`, `search_y_min`, `search_y_max`, `search_footprint_x`, `search_footprint_y`, and `search_overlap_ratio`.
- [ ] Add `std::vector<double> search_points_x` and `search_points_y`, initialized to `[-3, 3, 3, -3, -3, 3]` and `[-3, -3, 0, 0, 3, 3]`.
- [ ] Load arrays using `private_nh.getParam("search/points_x", search_points_x)` and the corresponding Y key, retaining defaults when either key is missing.
- [ ] In `validate()`, restore both default arrays if either is empty or their sizes differ. Preserve positive validation for `search_height` and `max_search_passes`.
- [ ] Change startup logging from rectangle/footprint details to feature-point count, height, and pass count.
- [ ] Replace old YAML geometry keys with `points_x` and `points_y` arrays.
- [ ] Commit with message `refactor: configure feature point search`.

### Task 2: Simplify CoverageSearchPlanner

**Files:**
- Modify: `src/mission_node/include/mission_node/CoverageSearchPlanner.h`
- Modify: `src/mission_node/src/CoverageSearchPlanner.cpp`

- [ ] Change `configure()` to accept `const std::vector<double>& points_x`, `const std::vector<double>& points_y`, `double height`, and `int max_passes`.
- [ ] Replace boundary, footprint, and overlap members with stored X/Y coordinate vectors.
- [ ] Remove scan-line generation, axis transposition, footprint offsets, and `makeScanLines()`.
- [ ] Implement `generate()` by clearing state and calling `addWaypoint(points_x_[i], points_y_[i])` once for each paired coordinate in its supplied order.
- [ ] Keep `empty()`, `currentWaypoint()`, `markCurrentVisited()`, `resumeNearest()`, `startNextPass()`, `passCount()`, and `exhausted()` unchanged. `startNextPass()` continues reversing the feature-point order.
- [ ] Commit with message `refactor: use direct feature point segments`.

### Task 3: Update MissionFSM configuration and publish

**Files:**
- Modify: `src/mission_node/src/MissionFsm.cpp`

- [ ] Update the constructor call to `search_planner_.configure(parameters_.search_points_x, parameters_.search_points_y, parameters_.search_height, parameters_.max_search_passes)`.
- [ ] Inspect the `SEARCHING` state to confirm it publishes `position_pub` only when `search_goal_sent_` is false and publishes no generated intermediate coordinates.
- [ ] Inspect source scope and stage only the parameter, planner, FSM, config, design, and plan files; exclude build/devel artifacts, bag files, editor files, and `.superpowers/`.
- [ ] Commit with message `refactor: connect mission to feature point search`.

### Task 4: Publish without verification claims

**Files:**
- Inspect committed changes only

- [ ] Review `git status --short` and commits without running build or tests.
- [ ] Push `add_random` to `origin` with a normal non-force push.
- [ ] Report the six default points and explicitly state that compilation, simulation, and hardware behavior were not verified.
