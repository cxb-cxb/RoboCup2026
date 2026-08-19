# Coverage Target Search Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace fixed target coordinates with parameterized coverage search that selects and drops three unique targets by the confirmed priority rules.

**Architecture:** `CoverageSearchPlanner` generates and tracks a resumable lawnmower route while `MissionFSM` remains responsible for ROS data, stable target estimation, priority scheduling, two-stage adjustment, cargo release, and transition into the unchanged post-delivery states. Existing ROS topics, EGO-Planner publishers, image messages, adjustment math, and serial commands remain unchanged.

**Tech Stack:** ROS 1 roscpp, C++11, geometry_msgs, catkin, YAML parameter server

**Verification constraint:** The user explicitly requested source-structure changes only. Do not install dependencies or run compilation, tests, simulation, or hardware verification; report this limitation on delivery.

---

### Task 1: Add search parameters

**Files:**
- Modify: `src/mission_node/include/mission_node/MissionParameters.h`
- Modify: `src/mission_node/src/MissionParameters.cpp`
- Modify: `src/mission_node/config/mission_params.yaml`

- [ ] Add fields `search_x_min`, `search_x_max`, `search_y_min`, `search_y_max`, `search_height`, `search_footprint_x`, `search_footprint_y`, `search_overlap_ratio`, and `max_search_passes` with defaults `-4`, `4`, `-4`, `4`, `1`, `2`, `1.5`, `0.25`, and `3`.
- [ ] Load the exact keys `search/x_min`, `search/x_max`, `search/y_min`, `search/y_max`, `search/height`, `search/footprint_x`, `search/footprint_y`, `search/overlap_ratio`, and `search/max_passes`.
- [ ] In `validate()`, restore both X bounds when `x_min >= x_max`, restore both Y bounds when `y_min >= y_max`, restore positive defaults for height and footprint, restore `0.25` unless overlap is in `[0, 1)`, and restore `3` unless max passes is positive. Emit `ROS_WARN` for each invalid group.
- [ ] Extend startup logging with the final search rectangle, height, footprint, overlap, and pass count.
- [ ] Add the confirmed `search` section to YAML.
- [ ] Commit only these parameter files with message `feat: configure coverage search area`.

### Task 2: Add the resumable coverage planner

**Files:**
- Create: `src/mission_node/include/mission_node/CoverageSearchPlanner.h`
- Create: `src/mission_node/src/CoverageSearchPlanner.cpp`
- Modify: `src/mission_node/CMakeLists.txt`

- [ ] Define `CoverageSearchPlanner::configure(double x_min, double x_max, double y_min, double y_max, double height, double footprint_x, double footprint_y, double overlap, int max_passes)` and store validated values supplied by `MissionParameters`.
- [ ] Define `generate()` to create `geometry_msgs::PoseStamped` row endpoints in frame `camera_init`. For the default square, sweep along X: use X endpoints `x_min + footprint_x/2` and `x_max - footprint_x/2`, generate Y rows from `y_min + footprint_y/2` through `y_max - footprint_y/2` using `footprint_y * (1-overlap)`, and reverse every second row. For a rectangle taller than wide, transpose this rule and sweep along Y.
- [ ] Define navigation methods `empty()`, `hasCurrentWaypoint()`, `currentWaypoint()`, `markCurrentVisited()`, `resumeNearest(const geometry_msgs::Point&)`, `startNextPass(const geometry_msgs::Point&)`, `passCount()`, and `exhausted()`. Track visited entries separately from the waypoint vector.
- [ ] `resumeNearest()` must select the nearest unvisited waypoint without changing visit flags. `startNextPass()` increments the pass count, reverses the waypoint order, clears visited flags, and selects the nearest endpoint. `exhausted()` returns true when pass count reaches the configured maximum.
- [ ] Add `src/CoverageSearchPlanner.cpp` to the existing `run_node` executable in CMake.
- [ ] Commit the planner and CMake change with message `feat: add resumable coverage planner`.

### Task 3: Add search mission data and helpers

**Files:**
- Modify: `src/mission_node/include/mission_node/MissionFsm.h`
- Modify: `src/mission_node/src/MissionFsm.cpp`

- [ ] Include `<map>`, `<set>`, and `CoverageSearchPlanner.h`. Add states `GENERATE_SEARCH`, `SEARCHING`, `APPROACH_DETECTED_TARGET`, `SEARCH_TARGET_DROP`, `RESUME_SEARCH`, and `SEARCH_FINISHED`.
- [ ] Add members: `CoverageSearchPlanner search_planner_`, `std::set<std::string> remaining_classes_`, `std::map<std::string, std::vector<std::pair<double,double>>> target_samples_`, `std::map<std::string, geometry_msgs::PoseStamped> cached_targets_`, `geometry_msgs::PoseStamped active_search_target_`, `std::string active_search_class_`, `int completed_drops_`, `int search_adjust_phase_`, `ros::Time search_phase_started_`, and `bool search_goal_sent_`.
- [ ] Initialize the remaining set with all six exact class strings, reset counters and flags, configure the planner from `parameters_`, and keep the existing three `C/U/P` cargo commands.
- [ ] Add helpers `isKnownTargetClass`, `isHighPriorityClass`, `recordSearchDetection`, `buildStableTarget`, `selectCachedLowPriorityTarget`, `beginSearchTarget`, `completeSearchDrop`, `cancelSearchTarget`, and `finishSearchMission`.
- [ ] `recordSearchDetection()` must ignore empty/unknown/already-dropped classes and invalid image centers, call existing `computeAdjustment()`, convert correction to `camera_init` world X/Y using current pose, append samples by class, and cap each class at the existing 15-sample threshold.
- [ ] `buildStableTarget()` must require 15 samples, independently median-sort X and Y, set Z to configured search height, set identity orientation and `camera_init` frame, write `cached_targets_`, and clear that class's samples.
- [ ] For stable high-priority classes call `beginSearchTarget()` immediately when no target is active. For stable low-priority classes only cache the pose.
- [ ] `selectCachedLowPriorityTarget()` must inspect `bridge`, then `tent`, then `bunker`, returning the first cached class still present in `remaining_classes_`.
- [ ] `completeSearchDrop()` must guard `Drop_queue.empty()`, call `Ser_pub(front)`, pop once, erase the active class from all containers, increment `completed_drops_`, clear active state, and choose `SEARCH_FINISHED` after three drops or `RESUME_SEARCH` otherwise.
- [ ] `cancelSearchTarget()` must keep the class in `remaining_classes_`, clear only its stale samples and active lock, return to `RESUME_SEARCH`, and never call `Ser_pub()`.
- [ ] `finishSearchMission()` must set `finish_Point` to the current X/Y at configured search height with identity orientation, set `ego_contral = true`, and transition to existing `FINISH_Dynamic`.
- [ ] Commit the state model and helpers with message `feat: track searched mission targets`.

### Task 4: Wire coverage search into the state machine

**Files:**
- Modify: `src/mission_node/src/MissionFsm.cpp`

- [ ] Change the successful takeoff/height transition from `TRACKING_WAYPOINT` to `GENERATE_SEARCH`. Leave old fixed-waypoint cases compiled but unreachable so unrelated downstream code is not broadly rewritten.
- [ ] In `GENERATE_SEARCH`, generate the route, start flight-data collection, select the nearest initial waypoint, and enter `SEARCHING`; if route generation is empty, log an error and enter `SEARCH_FINISHED`.
- [ ] In `SEARCHING`, call `recordSearchDetection()` every cycle, publish the current search waypoint through existing `position_pub` once and `pos_pub` continuously, mark it visited within `0.15 m`, and advance through the planner. A stable high-priority detection transitions immediately to `APPROACH_DETECTED_TARGET`.
- [ ] At the end of a pass, if fewer than three drops are complete, call `selectCachedLowPriorityTarget()`. If one exists, lock it and enter `APPROACH_DETECTED_TARGET`; otherwise start the next pass or enter `SEARCH_FINISHED` when the configured limit is reached.
- [ ] In `APPROACH_DETECTED_TARGET`, publish `active_search_target_` through the same EGO-Planner interface. Within `0.15 m`, set adjustment phase zero, start a timeout clock, and enter `SEARCH_TARGET_DROP`.
- [ ] In `SEARCH_TARGET_DROP`, preserve the existing two-stage behavior: phase zero uses the stable median position as the first adjustment at search height; after arrival and a fresh valid detection, phase one calls `computeAdjustment()`, applies cargo X offsets `C:+0.2`, `U:-0.2`, `P:0.0`, commands Z `0.2`, and republishes. When phase-one position is within `0.05 m`, call `completeSearchDrop()`. If the current target cannot be reacquired for five seconds, call `cancelSearchTarget()`.
- [ ] In `RESUME_SEARCH`, raise the current position back to configured search height. Once within `0.1 m` vertically, call `resumeNearest()` and return to `SEARCHING`.
- [ ] In `SEARCH_FINISHED`, log whether exit was caused by three completed drops or maximum passes and call `finishSearchMission()` exactly once.
- [ ] Ensure every `Drop_queue.front()` in the new path is preceded by an empty check and no class is removed before `Ser_pub()` is actually called.
- [ ] Commit state integration with message `feat: search and drop prioritized targets`.

### Task 5: Remove obsolete fixed-target entry behavior and inspect scope

**Files:**
- Modify: `src/mission_node/src/MissionFsm.cpp`
- Modify: `src/mission_node/include/mission_node/MissionFsm.h`

- [ ] Remove constructor initialization of the five fixed delivery `target_points`; preserve crossing points and post-delivery coordinates used by `FINISH_Dynamic`, `CROSS_LAND`, and later states.
- [ ] Remove `dropped_classes` initialization and search-path uses in favor of `remaining_classes_`; leave unrelated legacy functions compiled only if still referenced by unreachable legacy states.
- [ ] Correct target validation strings from `tant` to `tank` and include `tent` and `bunker` wherever legacy median filtering remains shared with the new path.
- [ ] Confirm by source inspection that `run_node.cpp`, subscriber topics, publisher topics, message types, `computeAdjustment()`, `Ser_pub()`, and the state sequence beginning at `FINISH_Dynamic` were not renamed or removed.
- [ ] Inspect `git diff` and stage only source, config, CMake, design, and plan files. Exclude build/devel artifacts, bag files, editor files, and `.superpowers/`.
- [ ] Commit cleanup with message `refactor: retire fixed delivery waypoints`.

### Task 6: Publish without claiming verification

**Files:**
- Inspect committed changes only

- [ ] Review `git status --short`, `git diff --stat` and the final commit list; do not run compilation or tests per user instruction.
- [ ] Push the current `add_random` branch to its configured `origin` using a normal non-force push.
- [ ] Report the pushed branch and commits, list the new configuration keys, and explicitly state that compilation, simulation, and hardware behavior were not verified.
