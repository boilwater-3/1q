# FD 开发期验证工具（fd_tools）

本目录是 flight_dynamic（机动）模块的**开发期验证工具**（2026-08-10 由
`examples/flight_dynamic/` 迁入，examples 层收敛为单一消费方参考角色）。

## 验证方法论：逐帧数据导出 + 脚本分析

机动模块的验证形态与其它模块不同：**六自由度轨迹的"对不对"很难用单断言
表达**（GTest 适合确定性不变式，不适合轨迹形状/收敛过程判断）。开发期
验证主要靠**导出每一帧的状态数据，用脚本/图表分析判断机动逻辑**：

| 形态 | 载体 | 适用场景 |
| --- | --- | --- |
| 确定性断言 | `tests/unit/flight_dynamic/` GTest 分区（如 fd_orbit_quality_test、fd_takeoff_substep_test） | 不崩溃/无 NaN、门限边界、积分步长契约 |
| **逐帧数据导出 + 分析** | 本目录 7 个可执行 + Python 脚本 | 轨迹形状、收敛过程、质量指标（半径误差/角速度一致性）、进场几何 |

本目录工具**不进 ctest**（与 FD 单测注释"示例只编译不运行、也不进 ctest"
的决策一致）：质量分析结果依赖人工判断，不设硬断言。工具随测试构建编译
（`ENABLE_TESTING` + `ONEQ_ENABLE_FLIGHT_DYNAMIC` 门控）。

> 开发期工具可使用 `src/` 内部头与 JSBSim 模型头（如 takeoff_land_csv
> 读取起落架/推进器状态）——这是模块开发期验证形态的合法需求，不代表
> 消费方集成方式；消费方集成参考见 `examples/`。

## 工具清单

| 工具 | 用途 | 输出 |
| --- | --- | --- |
| `takeoff_land_csv` | 起飞→巡航→降落完整机动链（46 列状态：起落架/推力/姿态） | CSV（逐帧） |
| `orbit_quality_csv` | 盘旋质量分析：半径精度/角速度一致性/滚转极限，多半径扫描 | CSV（每配置一行 27 指标） |
| `orbit_trace_csv` | 盘旋逐点轨迹（到中心距离/半径误差） | CSV（逐帧 13 列） |
| `racetrack_trace_csv` | 跑道形逐点轨迹（到跑道距离/误差） | CSV（逐帧 13 列） |
| `racetrack_approach_trace` | 跑道巡逻进场航迹（8 个进场场景） | 每场景一个 CSV |
| `figure8_approach_trace` | Figure-8 进场航迹（9 个进场场景，任意位置/方向进场质量） | 每场景一个 CSV |
| `sturn_trace_csv` | S 型机动逐点轨迹 | CSV（逐帧 10 列） |
| `aircraft_probe_csv` | 机型能力探针（环境变量门控导出：剖面/航点扫描/盘旋扫描，2026-08-10 自 GTest 迁出） | 3 种 CSV |

各工具独立 main（`<工具名> --help` 查看参数；机型参数：c172x / DHC6 /
737 / 747 / f16 等，`ALL` 遍历内置机型表）。构建产物在
`build/<preset>/bin/`。

## 分析脚本

- `orbit_visualize.py`（本目录）：读 orbit_trace_csv 输出 → KML（Google
  Earth）+ matplotlib 四面板图命令（轨迹/期望圆/误差/高度剖面）；
- `tools/fd_trace_to_viz.py`（仓库 tools/）：把各 `*_trace_csv` 的轨迹
  CSV 归一为统一可视化契约 v2（`platform_track.csv`），产物交给
  `examples/common/viz/build_viewer.py` 生成共享 HTML 查看器；
- `tools/plot_maneuvers.py` / `tools/plot_figure8_approach.py` /
  `tools/plot_racetrack_approach.py`：进场/机动静态图（matplotlib）。

## 与单测的分工

| 场景 | 归属 |
| --- | --- |
| 起飞段积分子步进契约（dt=0.01 稳定 / 0.1 发散） | `fd_takeoff_substep_test`（2026-08-10 自 takeoff_land_csv 迁入，证据锚点） |
| 轨道机动快速断言（不崩溃/无 NaN/高度漂移/滚转越界） | `fd_orbit_quality_test` |
| 机型能力/航点/盘旋探针 CSV 导出（数据导出非断言） | 本目录 `aircraft_probe_csv`（环境变量门控，2026-08-10 自 fd_aircraft_probe_test 迁出） |
| 重质量分析（半径精度、角速度一致性等） | 本目录 `orbit_quality_csv`（fd_orbit_quality_test 文件头注释指向） |
