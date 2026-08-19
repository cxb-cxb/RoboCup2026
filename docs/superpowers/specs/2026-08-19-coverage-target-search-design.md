# 无坐标靶标覆盖搜索与投放设计

## 目标

将现有依赖固定靶标坐标的投放流程改为自主覆盖搜索。无人机在固定高度沿参数化搜索航线飞行，由现有识别接口发现目标，由 EGO-Planner 实时规划和避障，并对三个目标执行现有二次微调与串口投放。投放结束后的穿越、返航和降落流程保持不变。

## 任务约束

- 场地以无人机起飞点为原点，搜索区域为无禁飞区的矩形。
- 默认测试区域为 `[-4, 4] × [-4, 4] m`。
- 默认搜索高度为 `1.0 m`。
- 默认有效扫描范围为 X 方向 `2.0 m`、Y 方向 `1.5 m`。
- 相邻扫描区域默认重叠 `25%`。
- 所有搜索目标都通过现有 EGO-Planner 接口发布，由规划器负责实时避障。
- 识别类别固定为 `random`、`tank`、`car`、`bridge`、`tent`、`bunker`。
- 机载货物只有三个，每个类别最多投放一次。
- `random`、`tank`、`car` 为并列第一优先级；剩余优先级为 `bridge`、`tent`、`bunker`。

## 参数服务器

在现有 `mission_params.yaml` 和 `MissionParameters` 中增加：

```yaml
search:
  x_min: -4.0
  x_max: 4.0
  y_min: -4.0
  y_max: 4.0
  height: 1.0
  footprint_x: 2.0
  footprint_y: 1.5
  overlap_ratio: 0.25
  max_passes: 3
```

边界必须满足最小值小于最大值，高度和覆盖尺寸必须为正数，重叠率必须位于 `[0, 1)`，最大搜索遍数必须大于零。非法参数恢复为上述默认值并输出 ROS 警告。

## 覆盖航线

新增内部类 `CoverageSearchPlanner`，根据矩形边界、有效覆盖范围和重叠率生成往复式割草机航线。默认有效步距为：

- X 方向：`footprint_x × (1 - overlap_ratio) = 1.5 m`
- Y 方向：`footprint_y × (1 - overlap_ratio) = 1.125 m`

无人机中心航点从边界向内偏移半个有效扫描范围，使搜索覆盖到场地边缘。航线沿较长方向连续飞行，在行末换向；默认正方形场地沿 X 方向飞行，每一行由左右端点两个特征航点定义，行间按 Y 方向有效步距排列。只有改为沿 Y 方向飞行时才使用 X 方向有效步距排列扫描列。规划器仅生成 `geometry_msgs::PoseStamped` 目标点，不直接发布消息或控制飞行。

规划器记录当前航点、已访问航点和搜索遍数。目标投放中断搜索时保留进度；投放完成后，从距离无人机当前位置最近的未访问航点恢复，并继续既定往复式顺序。新一遍覆盖反转航线方向，减少返回起点的无效航程。最多执行三遍。

## 目标记录和优先级

`MissionFSM` 维护：

- `remaining_classes`：尚未投放的有效类别集合。
- `target_samples`：每个类别的世界坐标样本。
- `cached_targets`：稳定识别后计算得到的类别目标坐标。
- `active_target`：当前唯一锁定的投放目标。

搜索过程中继续通过现有识别消息、位姿和 `computeAdjustment()` 计算目标世界坐标。单帧识别不触发任务切换；只有取得足够样本并计算中位数坐标后，目标才视为稳定。

稳定识别到 `random`、`tank` 或 `car` 时，立即暂停覆盖搜索并锁定该目标。稳定识别到 `bridge`、`tent` 或 `bunker` 时，只更新缓存，不中断第一阶段搜索。当前目标接近和微调期间可以缓存其他类别，但禁止抢占 `active_target`。

投放成功后，从 `remaining_classes` 和 `cached_targets` 删除对应类别，并从 `Drop_queue` 弹出一个货物指令。已删除类别的后续识别结果全部忽略。

## 投放流程

每个锁定目标执行：

1. 通过现有目标发布接口将稳定中位数坐标交给 EGO-Planner。
2. 到达目标附近后，使用现有检测框中心、深度、姿态和 `computeAdjustment()` 完成第一次微调。
3. 无人机稳定并重新采样后再次计算修正位置，完成第二次微调。
4. 第二次微调满足位置阈值且 `Drop_queue` 非空时，调用现有 `Ser_pub(Drop_queue.front())` 投放。
5. 清理当前类别临时样本，删除类别，恢复覆盖搜索。

目标接近过程中短暂丢失时继续前往已经稳定估计的坐标。到达后若持续无法重新识别，则取消本次投放，保留该类别为未完成并恢复搜索，禁止误投。

## 搜索结束与低优先级补投

任一时刻完成三次投放后立即结束搜索。完整覆盖一遍但投放数不足三时，按 `bridge → tent → bunker` 从已缓存目标中选择并投放，直到货物耗尽或缓存为空。

如果投放数和缓存目标仍不足三，则开始下一遍覆盖。达到 `max_passes` 后仍不足三时，输出 ROS 错误并结束搜索。正常完成和超限退出均衔接现有 `FINISH_Dynamic → CROSS_LAND → DECIDE_CROSS...` 后续流程。

## 状态机和代码边界

`MissionFSM::DroneState` 增加覆盖搜索所需状态：

- `GENERATE_SEARCH`
- `SEARCHING`
- `APPROACH_DETECTED_TARGET`
- `SEARCH_TARGET_DROP`
- `RESUME_SEARCH`
- `SEARCH_FINISHED`

新增文件：

- `include/mission_node/CoverageSearchPlanner.h`
- `src/CoverageSearchPlanner.cpp`

修改文件：

- `include/mission_node/MissionFsm.h`
- `src/MissionFsm.cpp`
- `include/mission_node/MissionParameters.h`
- `src/MissionParameters.cpp`
- `config/mission_params.yaml`
- `CMakeLists.txt`

外部 ROS 话题、消息类型、EGO-Planner 发布接口、串口投放接口和投放后的穿越/降落状态不变。失效的固定目标航点、`mission_num` 固定靶标索引，以及分别处理 random、tank 和普通目标的重复投放分支由统一搜索调度替代。旧拼写 `tant` 修正为 `tank`，并补全 `tent`、`bunker`。

## 验证责任

本次按用户要求只修改代码结构，不在当前环境安装依赖或执行编译、仿真和实机验证。用户后续验证重点包括：航点覆盖范围、EGO-Planner 避障、稳定识别阈值、搜索中断和恢复、三次投放、类别去重、二次微调、最大遍数退出及后续状态衔接。
