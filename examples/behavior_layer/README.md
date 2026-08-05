# 行为层参考实现（`examples/behavior_layer/`）

消费方业务层的 EnTT ECS 参考实现（冻结契约：`docs/review/Bahavior.md` §5）。
实体/组件装配由 `entt::registry` 承担，逻辑以纯数据组件 + 自由函数系统表达；
**EnTT 仅为 example 侧依赖**（`conanfile.py` 基础清单，header-only），不进入库本体，
`include/1q/` 公共头与 C++11 下限均不受影响。

当前形态：**三传感器（AR / ESR / EOS）单平台端到端全链**——三会话同周期推进、
输出在边界适配为泛型探测记录、一次融合引擎更新产出跨源合并态势、决策下发命令帧、
ECM 输入帧由 ESR 观测填充、平台动力学由 `flight_system` 驱动（`ONEQ_ENABLE_FLIGHT_DYNAMIC`
开关注入 JSBSim 真实飞行仿真，默认 OFF 回退运动学）。

## 目录结构

| 文件 | 职责 |
| --- | --- |
| `components.h` | 六个数据组件（纯数据、无虚函数）+ 源通道标识常量 |
| `systems.h` / `systems.cpp` | 五个系统（自由函数，`entt::registry&`）+ 三会话驱动与适配 |
| `flight_system.h` / `flight_system.cpp` | 平台动力学（`RoutePlan` → `FlightManager` 适配 + 运动学回退；唯一含 FD 头的文件） |
| `assembly.h` / `assembly.cpp` | 装配：registry ctx 上下文、实体创建、周期调用序、观察者工厂 |
| `behavior_layer_demo.cpp` | 主程序：脚本化场景 + 每周期系统调用序 + 事件报告 |
| `viz_recorder.h` / `viz_recorder.cpp` | 可视化数据记录器：逐周期把飞行/传感器/融合数据导出为 8 个 CSV |
| `build_viewer.py` | 读取 CSV 构建单文件交互式 HTML 查看器（vanilla JS + SVG，无 CDN） |

## 组件（`components.h`）

| 组件 | 内容 | 写入方 |
| --- | --- | --- |
| `TaskingComponent` | 角色（单机/长机/僚机）、上下级、区域任务（`navigation` 面类型） | 装配层（层级显式注入，无"发现"机制） |
| `SensorObservationComponent` | `source_id` + 泛型探测记录（`fusion::DetectionRecord`） | `recon_system`（每传感器实体一份） |
| `FleetStatusComponent` | 平台 LLA 位置/航向/速度 | `flight_system`（长机动力学推进后同步三传感器实体） |
| `RoutePlanComponent` | 航路计划（`navigation::RoutePlan`）+ 版本号 | `maneuver_system`（推进 `next_index` 属 `flight_system`） |
| `FusedSituationComponent` | 融合态势 + 新/消失事件计数 | `recon_system`（经 `fusion` 引擎） |
| `CommandFrameComponent` | AR 战术指令 / ECM 周期输入 / 外部决策覆盖 | `jam_system` + `decision_system` |

实体挂载：长机（任务/航路/融合/命令帧/编队状态）+ 2 僚机（任务/航路/编队状态）+
**三传感器实体**（`SensorObservationComponent` + `FleetStatusComponent`，
`source_id` = 1/2/3 区分通道）。组件类型零新增——多传感器接入靠实体拆分与
`source_id` 分派，不靠新组件类型。

## 系统与周期调用序

每周期由 `StepBehaviorLayer()` 按以下顺序执行（对齐 session `Step` 语义）：

1. **`flight_system`** — 长机承载平台动力学（层级显式输入：wingman 有上级 →
   零计算，冻结契约 §5）。`ONEQ_ENABLE_FLIGHT_DYNAMIC=ON` 时经 `FlightManager`
   （JSBSim c172x）真实飞行仿真：航路版本变化时把 `RoutePlanComponent` 剩余航点
   适配为 `kFlyToWaypoint` 机动队列（deg→rad 属业务层适配职责，`RoutePoint.h`），
   每行为周期子步进 10×100 ms，`VehicleState` 映射回编队状态并按几何到达推进
   `next_index`；关闭或初始化失败（aircraft 数据缺失/配平失败）时回退运动学
   近似（原 demo `AdvancePlatform` 逻辑）。末段把长机位姿同步到三传感器实体，
   recon 本周期即看到推进后的平台；
2. **`recon_system`** — 按实体 `source_id` 分派三会话（同周期同时间戳，多源时间
   对齐即业务层职责）：
   - **AR**：`StepWithResult` → `ArCycleOutputAdapter` → `DetectionRecord`
     （key = `association_key`，带位置）；
   - **ESR**：`StepWithResult` → `EmitterHypothesisList` 适配
     （key = `hypothesis_id`，方位 + 归一化射频特征 {GHz/MHz/ms/µs}，
     quality = 假设置信度）；
   - **EOS**：`EosCycleInputAdapter::Build` → `StepWithResult` → `EosDetectionRecord`
     适配（key = 0 无身份，**仅方位通道**，quality = SNR 归一化；range 通道首期
     不使用，保留方位相干关联路径）；
   
   三实体探测聚合后一次 `FusionEngine::Update` 更新融合态势；
3. **`maneuver_system`** — 层级纯函数：有上级 → 零计算；长机/单机调
   `AreaCoveragePlanner::Plan` 规划区域覆盖航路；长机把计划下发到各僚机；
4. **`jam_system`** — 构造 `EcmCycleInput`，ESR 成功周期发布去真值化观测帧时
   填充 `sensor_observation_frame`（`source_esr_batch_id` = ESR 批次号；观测字段
   业务层映射：RF 参数五件套直映、`EsrObservationQuality` → 威胁分数、
   SNR → 置信度）。无逐威胁 tasking SPI（冻结契约 §5/§7）；
5. **`decision_system`** — 聚合融合态势（高置信威胁判定）产出 `ArCommand`，
   写入命令帧组件；`external_decision` 为预留接线位。

事件模型：命令 = 写 `CommandFrameComponent`；事件报告 = `entt::observer`
（`MakeSituationObserver`，监听 `FusedSituationComponent` 变化），不建全局事件总线。

## 多源关联路径（demo 实际演示的融合语义）

| 关联层 | demo 中的表现 |
| --- | --- |
| ① 身份键直挂 | AR `association_key` 与 ESR `hypothesis_id` 均为从 1 递增的库内键，同物理目标恰好同键 → 同键探测归并（`key=1/2/3` 双通道航迹，置信度 = Σ 判决×质量×权重 跨源累加）。跨源一致性由调用方保证（冻结契约 §4.1）——demo 中两模块键空间巧合对齐 |
| ② 位置半径（KD-tree） | AR 轨迹带位置；同周期内多 AR 航迹互不合并（冻结：同周期新建航迹不参与关联） |
| ③ 方位相干 | EOS 无身份仅方位探测 → 与上一周期已存在的 ESR 方位航迹做 `AreBearingsCoherent`（方位最短弧 + 俯仰线性，均 < 门限 8°）→ 并入（`ch[2:x 3:y]` 双通道）；不相干时生成合成键（≥ 2^63）新航迹，失跟超限后删除 |
| ④ 特征门限 | 未启用（ESR 记录携带特征但异构维度下引擎本就不约束，不虚设门） |

**已知模块行为（非融合缺陷）**：ESR 假设关联器会随时间换发假设 id（3 个辐射源
产生 5-8 条假设航迹），融合航迹随之轮换——这是 ESR 模块自身的假设管理行为；
EOS 探测只在其扫描经过目标时产生（每 4 周期约 1-2 次），并入当周期对应的 ESR
航迹。航迹锚点 = 最近一次量测（任意源），方位比较针对该锚点。

**坐标参考**：三会话共享零姿态平台局部系（`atan2(north, east)`，az 0 = 东），
因此 ESR 假设方位与 EOS 探测方位可直接相干比较。

## 接线位（执行面驱动）

| 执行面 | 接线 | 状态 |
| --- | --- | --- |
| AR | `ArSession::SubmitExternalDecision` / `decision_observation` | 已演示（命令帧输出；外部覆盖为预留位） |
| ECM | `EcmCycleInput`（`sensor_observation_frame` 由 ESR 观测填充） | 已演示（帧构造 + 观测映射；ECM 会话驱动属消费方职责） |
| ESR | 会话输出 → `DetectionRecord`（source_id=2） | 已接入 |
| EOS | 会话输出 → `DetectionRecord`（source_id=3，仅方位） | 已接入 |
| navigation | `AreaCoveragePlanner::Plan` → `RoutePlanComponent` | 已演示 |
| fusion | `FusionEngine::Update` → `FusedSituationComponent` | 已演示（跨源合并） |
| flight_dynamic | `flight_system`：`RoutePlanComponent` → `kFlyToWaypoint` 机动队列 → `FlightManager`（c172x） | 已接入（`ONEQ_ENABLE_FLIGHT_DYNAMIC=ON` 时真实飞行仿真；关闭/数据缺失回退运动学） |

## 场景调参说明（业务层决策，共享 JSON 未改动）

- **平台 700 m 高度**：使 EOS 探测距离窗（`alt/sin(视轴±半视场)`）落在
  [10, 40] km，与目标斜距（16-20 km）匹配；
- **平台标称速度 65 m/s**（c172x 巡航量级）：FD 就绪时装配层以性能面实际巡航
  覆盖（`GetControlProfile().cruise_speed_mps` ≈ 76 m/s），实际飞行 ~54-65 m/s；
  运动学回退路径沿用标称值；
- **覆盖区域与航点间距按飞行动力学设计**：c172x 转弯半径
  `v²/(g·tan(最大坡度))` ≈ 1.1-1.5 km（65-76 m/s）。中间航点（队列后继仍为
  kFlyToWaypoint）按导航语义完成（法平面穿越 / 到达半径 max(半径, 100 m)），不再要求
  间距大于捕获圈；最终航点仍按转弯量级捕获圈（≈1.7-2.3 km）提前到达。区域按
  飞行可见性调为 (30±0.0135, 120.05..0.081)（约 3 km × 3 km，起点距平台 ~4.8 km）、
  扫描线距 3000 m、演示周期 200（~65 m/s × 200 s ≈ 13 km，可见起飞巡航 → 到达区域 →
  航点推进 0/2 → 1/2 → 2/2）；
- **目标脚本**：3 目标正东（北偏东 90°）16/18/20 km，`v_east` 58-62 m/s 与平台
  同速东移 → 相对斜距全程稳定在 EOS 距离窗与扫描覆盖（±40°）内，三传感器
  全程同见（EOS 探测 SNR ≈ 18-20 dB）；
- **EOS 会话配置覆写**（`configs.eos.mission`，仅内存）：`frame_rate_hz` 30→10
  （周期校验要求 dt ≤ 10/帧率，演示按 1 s/周期推进）；扫描几何从"下视地面监视
  （az ±10°、俯仰中心 -48°）"覆写为"水平扫描（az ±40°、20°/s）"以匹配空中目标；
- **ESR 辐射源用脉冲列波形**（200 脉冲 @ 9.5/10.0/10.5 GHz 互异频率）：噪声波形
  有效脉冲数为 1，在 `pfa=1e-6` 统计检测门限下无法过检；互异频率保证分选聚簇
  稳定分离 3 条辐射源假设；
- **飞行子步进 100 ms**（10 步/周期）：c172x 巡航段积分稳定（fd 测试用 10-20 ms
  覆盖机动段；本示例仅巡航 + 航点转弯），且 200 周期 × 10 步 = 2000 步使
  debug 冒烟运行时可控。

## 可视化（飞行轨迹 + 传感器日志）

示例默认把每周期可观测数据落盘为 CSV（默认 `/tmp/behavior_layer_viz`，
可用 `--output-dir <dir>` 覆盖），再由 `build_viewer.py` 构建**单文件交互式
HTML 查看器**（vanilla JS + SVG，无 CDN、离线可用）：

```bash
# 1. 运行示例（产物 8 个 CSV）
./build/llvm-ninja-release-local/bin/behavior_layer_demo --output-dir /tmp/behavior_layer_viz
# 2. 构建查看器（默认输出 <data_dir>/behavior_layer_viewer.html）
python3 examples/behavior_layer/build_viewer.py /tmp/behavior_layer_viz
# 3. 打开
open /tmp/behavior_layer_viz/behavior_layer_viewer.html
```

### CSV schema（`viz_recorder` 导出，周期号 = 跨表对齐主键，t_sec = cycle × 1 s）

| 文件 | 列 | 内容 |
| --- | --- | --- |
| `platform_track.csv` | `cycle,t_sec,lat_deg,lon_deg,alt_m,heading_deg,speed_mps,wp_index,wp_count,model` | 平台轨迹；`model` = `jsbsim`（真实飞行仿真）/ `kinematic`（运动学回退） |
| `target_truth.csv` | `cycle,t_sec,target_id,lat_deg,lon_deg,alt_m,rcs` | 世界真值目标（ECEF → LLA） |
| `ar_tracks.csv` | `cycle,t_sec,key,target_id,status,pos_x_m,pos_y_m,pos_z_m,speed_mps,rcs,hit_count,miss_count` | AR 航迹快照（雷达局部 ENU，status = tentative/confirmed/lost） |
| `eos_detections.csv` | `cycle,t_sec,det_id,target_id,range_m,az_deg,el_deg,snr_db,detected` | EOS 探测（`target_id` 来自仿真归属映射，非真实传感器输出） |
| `esr_hypotheses.csv` | `cycle,t_sec,hyp_id,bearing_az_deg,bearing_el_deg,confidence,mode,threat_level,last_seen_cycle` | ESR 辐射源假设（模式/威胁等级为可读字符串） |
| `fused_tracks.csv` | `cycle,t_sec,key,confidence,last_update_cycle,ar_samples,esr_samples,eos_samples,lat_deg,lon_deg,alt_m,bearing_az_deg` | 融合态势（各通道采样数；位置取 AR 通道量测、方位取 ESR 通道量测，缺失为空） |
| `route_plan.csv` | `index,lat_deg,lon_deg,alt_m,speed_mps,radius_m` | 航路计划（首次规划后写一次） |
| `waypoint_events.csv` | `t_sec,waypoint_index,intermediate,gate,distance_m,cross_track_m,along_track_m,threshold_m` | 航点完成事件（仅 `ONEQ_ENABLE_FLIGHT_DYNAMIC=ON` 时有数据；gate = within_radius/plane_crossing/fly_past） |

### 查看器面板

- **俯视地图**（本地 ENU，原点 = 平台初始位置）：平台轨迹（截至当前周期的高亮段 +
  航向指示线）、编号航路点（含到达半径与完成事件标注）、目标真值轨迹（虚线）、
  融合目标全周期点位；当前周期的 AR 航迹点（方块）、EOS 距离-方位射线、
  ESR 方位线随滑杆/播放联动。悬停任意元素显示详情；
- **传感器日志时间线**（Gantt）：每行 = 真值目标（T1001/T1002/T1003）或融合键
  （采样数 ≥ 下限，可调），色段 = 该通道在对应周期看到目标（AR 蓝 / EOS 绿 /
  ESR 橙 / 融合红），▲ 新目标 ▼ 消失标记；采样数低于下限的短命融合键聚合为
  churn 行（逐周期活跃计数）；
- **融合置信度**：稳定键的 confidence 演化折线 + 各通道采样量堆叠面积；
- **飞行剖面**：平台高度 / 速度 / 航向三条子图。

### 数据能回答什么

- **飞行器轨迹**：`platform_track.csv` 的 `model` 列直接区分 JSBSim 真实飞行
  （高度/速度随巡航与转弯连续变化）与运动学回退（高度恒 700 m、速度恒 65 m/s）；
- **传感器日志逻辑**：时间线直接呈现"谁在何时看到谁"——EOS 间歇探测（扫描
  经过目标时才有色段，README §场景调参 所述）、ESR 假设 id 轮换（辐射源假设
  关联器行为）、融合键 churn（短命合成键）均可见；
- **航点完成语义**：`waypoint_events.csv` 记录命中门（中间航点
  within_radius/plane_crossing vs 最终航点捕获圈）与距离/侧距/沿航迹/阈值。

## 构建与运行

```bash
bash scripts/bootstrap_conan.sh llvm-ninja-release-local   # 拉取 entt/3.14.0
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON
cmake --build --preset llvm-ninja-release-local --target behavior_layer_demo
./build/llvm-ninja-release-local/bin/behavior_layer_demo
```

启用 `BUILD_TESTING` 时注册冒烟测试 `examples::behavior_layer_demo`
（LABELS：`examples;behavior_layer`），验证 EnTT 依赖链与三传感器端到端全链；
Python3 可用时另注册 `examples::behavior_layer_viewer_check`（对 demo 产物跑
`build_viewer.py --check`，回归 CSV 结构/表头与统一 ENU 原点不变量）。

接入真实飞行仿真（可选）：

```bash
# 需 third_party/jsbsim aircraft 数据（nightly 浅克隆提供；本地可
# git clone --depth 1 https://github.com/JSBSim-Team/jsbsim.git third_party/jsbsim）
cmake --preset llvm-ninja-release-local -DENABLE_EXAMPLES=ON -D ONEQ_ENABLE_FLIGHT_DYNAMIC=ON
```

## 演进路线

ECS 组件/系统模式覆盖了 session_usage（API 教程）与 scene（端到端场景）类目的
职责；**三域（AR/ESR/EOS）per-domain 旧示例已于 2026-08-05 删除**，功能并入本示例；
三域配置加载器迁移至 `examples/common/config_loaders/<域>/`（供本示例与
batch_validation 共用）。**flight_dynamic 已于 2026-08-05 接入**（`flight_system`
适配，`ONEQ_ENABLE_FLIGHT_DYNAMIC` 开关门控，默认 OFF 回退运动学）；
SAR 示例与 batch_validation 保持不变。
