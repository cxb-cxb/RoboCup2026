# Mission Node 参数服务器设计

## 目标

将 `mission_node` 的关键初始化参数从 `MissionFsm.cpp` 中移出，集中保存到 YAML 配置文件，并由 ROS 1 参数服务器在 `run_node` 启动时提供。修改保持现有默认行为，同时允许不重新编译即可调整参数。

## 参数范围

本次仅配置以下参数：

- 相机内参：`fx`、`fy`、`cx`、`cy`
- 相机相对机体的偏移：`dx`、`dy`、`dz`
- 速度限制：`max_velocity_x`、`max_velocity_y`
- 投放模型：`drop_height`、`gravity`、`land_velocity`
- 历史数据窗口：`history_duration`

航点、状态机初始状态、采样数量和其他业务常量不在本次修改范围内。

## 文件与加载方式

- 新建 `src/mission_node/config/mission_params.yaml`，保存上述参数及说明。
- 新建 `src/mission_node/launch/mission_node.launch`。
- launch 文件在 `run_node` 的私有命名空间中加载 YAML，然后启动节点。
- `MissionFSM` 构造函数使用私有 `ros::NodeHandle` 读取参数，避免与其他节点的同名参数冲突。
- `CMakeLists.txt` 安装 `launch/` 和 `config/` 目录，支持安装空间运行。

## 代码结构

原有文件级全局常量改为 `MissionFSM` 的私有成员。成员在构造函数开始阶段加载，保证状态机初始化和首次调用 `process()` 前参数已经可用。参数读取保留当前硬编码值作为默认值，因此直接运行 `run_node` 而不经过 launch 时仍保持现有行为。

`drop_time` 不作为独立配置项；它在参数加载和校验完成后由 `drop_height / land_velocity` 计算，防止配置值之间不一致。

## 校验与日志

以下参数必须为正数：`fx`、`fy`、`max_velocity_x`、`max_velocity_y`、`drop_height`、`gravity`、`land_velocity` 和 `history_duration`。若参数非法，节点输出 ROS 警告并对该参数恢复现有默认值。坐标中心和相机偏移允许为零或负数。

节点启动时输出最终采用的参数，便于确认 YAML 是否已成功加载。

## 验证

- 添加自动化测试或静态配置检查，确认 YAML、launch 参数命名和 C++ 读取名称一致。
- 验证缺少参数时使用默认值，非法正数参数会回退，派生的 `drop_time` 使用校验后的值。
- 编译 `mission_node`，确认成员替换没有引入编译错误。
- 检查 launch XML 和 YAML 语法。

