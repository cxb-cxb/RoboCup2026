# Mission Parameter Server Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move the selected `mission_node` initialization constants into a YAML file loaded through the ROS 1 parameter server.

**Architecture:** A focused `MissionParameters` value type owns defaults, private-namespace loading, validation, derived `drop_time`, and startup logging. `MissionFSM` owns one instance and replaces the selected file-scope constants with its values; a launch file loads YAML into `run_node`'s private namespace.

**Tech Stack:** ROS 1 roscpp, catkin, YAML/roslaunch XML, GoogleTest

---

### Task 1: Parameter model and validation

**Files:**
- Create: `src/mission_node/include/mission_node/MissionParameters.h`
- Create: `src/mission_node/src/MissionParameters.cpp`
- Create: `src/mission_node/test/mission_parameters_test.cpp`
- Modify: `src/mission_node/CMakeLists.txt`

- [ ] **Step 1: Write failing tests**

Add GoogleTests proving that defaults match the former constants, valid private ROS parameters are loaded, invalid positive-only parameters revert to defaults, and `drop_time()` equals validated `drop_height / land_velocity`.

- [ ] **Step 2: Verify the tests fail for the missing type**

Run from the workspace root:

```bash
catkin_make run_tests_mission_node_gtest_mission_parameters_test
```

Expected: compilation failure because `MissionParameters.h` does not exist.

- [ ] **Step 3: Implement the minimal parameter type**

Define public numeric fields with the existing values as defaults, `void load(const ros::NodeHandle& private_nh)`, `void validate()`, `double dropTime() const`, and `void log() const`. Load these exact private keys: `camera/fx`, `camera/fy`, `camera/cx`, `camera/cy`, `camera/offset_x`, `camera/offset_y`, `camera/offset_z`, `control/max_velocity_x`, `control/max_velocity_y`, `drop/height`, `drop/gravity`, `drop/land_velocity`, and `filter/history_duration`.

Validation restores the original default for non-positive `fx`, `fy`, both velocity limits, drop height, gravity, land velocity, and history duration, emitting `ROS_WARN_STREAM` for each replacement. `dropTime()` performs the division only after validation.

- [ ] **Step 4: Build and run the focused tests**

```bash
catkin_make run_tests_mission_node_gtest_mission_parameters_test
catkin_test_results build/mission_node
```

Expected: all parameter tests pass with zero failures.

- [ ] **Step 5: Commit the unit**

```bash
git add src/mission_node/include/mission_node/MissionParameters.h src/mission_node/src/MissionParameters.cpp src/mission_node/test/mission_parameters_test.cpp src/mission_node/CMakeLists.txt
git commit -m "feat: add validated mission parameters"
```

### Task 2: Integrate parameters into MissionFSM

**Files:**
- Modify: `src/mission_node/include/mission_node/MissionFsm.h`
- Modify: `src/mission_node/src/MissionFsm.cpp`
- Modify: `src/mission_node/CMakeLists.txt`

- [ ] **Step 1: Add a failing source-level integration check**

Extend `mission_parameters_test.cpp` with a check that reads `MissionFsm.cpp` through a compile definition containing the source path and asserts the selected old file-scope constant declarations are absent. This guards against accidentally leaving two sources of truth.

- [ ] **Step 2: Verify the integration check fails**

Run the focused test target and confirm failure reports the existing `const double fx` declaration.

- [ ] **Step 3: Replace selected globals with the owned parameter object**

Include `MissionParameters.h`, add `MissionParameters parameters_;` to `MissionFSM`, and at the start of its constructor create `ros::NodeHandle private_nh("~")`, then call `parameters_.load(private_nh)`, `parameters_.validate()`, and `parameters_.log()`.

Replace uses of the selected constants with matching fields. Replace `drop_time` uses with `parameters_.dropTime()`. Do not alter waypoints, state initialization, sample counts, queue contents, or unrelated user changes.

Add `src/MissionParameters.cpp` to the `run_node` target and provide the source path compile definition only to the test target.

- [ ] **Step 4: Run the test and compile the package**

```bash
catkin_make run_tests_mission_node_gtest_mission_parameters_test
catkin_make --pkg mission_node
catkin_test_results build/mission_node
```

Expected: test and build exit successfully with zero test failures.

- [ ] **Step 5: Commit the integration**

```bash
git add src/mission_node/include/mission_node/MissionFsm.h src/mission_node/src/MissionFsm.cpp src/mission_node/CMakeLists.txt src/mission_node/test/mission_parameters_test.cpp
git commit -m "refactor: load mission settings from parameter server"
```

### Task 3: YAML, launch, and install rules

**Files:**
- Create: `src/mission_node/config/mission_params.yaml`
- Create: `src/mission_node/launch/mission_node.launch`
- Create: `src/mission_node/test/config_files_test.py`
- Modify: `src/mission_node/CMakeLists.txt`

- [ ] **Step 1: Write a failing configuration consistency test**

Create a Python unittest using only the standard library. It must assert both files exist, parse launch XML with `xml.etree.ElementTree`, confirm the node is package `mission_node`, type `run_node`, name `run_node`, and confirm its child `rosparam` loads `$(find mission_node)/config/mission_params.yaml`. It must also check the YAML contains each exact key listed in Task 1.

- [ ] **Step 2: Run it and verify the missing-file failure**

```bash
python3 src/mission_node/test/config_files_test.py
```

Expected: FAIL because the config and launch files do not exist.

- [ ] **Step 3: Add configuration and launch files**

Write YAML using the current values: camera `1092.34009`, `1088.58832`, `657.880369`, `361.681183`, offsets `0.0`, `0.21`, `0.0`; velocity limits `0.5`, `0.5`; drop height `1.0`, gravity `9.8`, land velocity `0.2`; history duration `5.0`.

Write a launch file whose `node` contains:

```xml
<rosparam command="load" file="$(find mission_node)/config/mission_params.yaml"/>
```

Add catkin install rules for the `launch` and `config` directories and register the Python test under `CATKIN_ENABLE_TESTING`.

- [ ] **Step 4: Verify configuration and installation**

```bash
python3 src/mission_node/test/config_files_test.py
catkin_make --pkg mission_node
catkin_make install --pkg mission_node
```

Expected: the test passes, the package builds, and both directories appear under `install/share/mission_node`.

- [ ] **Step 5: Commit configuration support**

```bash
git add src/mission_node/config/mission_params.yaml src/mission_node/launch/mission_node.launch src/mission_node/test/config_files_test.py src/mission_node/CMakeLists.txt
git commit -m "feat: launch mission node with YAML configuration"
```

### Task 4: Final verification and publication

**Files:**
- Verify all files changed by Tasks 1-3

- [ ] **Step 1: Run full relevant verification**

```bash
catkin_make --pkg mission_node
catkin_make run_tests_mission_node
catkin_test_results build/mission_node
python3 src/mission_node/test/config_files_test.py
git diff --check HEAD~3..HEAD
```

Expected: compilation and tests succeed, test results report zero failures, and diff check prints no errors.

- [ ] **Step 2: Inspect change scope**

Use `git status --short`, `git diff --stat HEAD~3..HEAD`, and `git log --oneline -4` to confirm only intentional source/config/test/document changes are committed; keep existing build artifacts and unrelated user edits out of commits.

- [ ] **Step 3: Configure and verify the publication remote**

Inspect current remotes and the default branch of `https://github.com/cxb-cxb/RoboCup2026.git`. Add a clearly named remote if necessary. Fetch before pushing and stop if histories conflict rather than force-pushing.

- [ ] **Step 4: Push the current branch safely**

Push the implementation commits to the matching remote branch using a normal, non-force push. Report the remote and branch actually updated.

