---
Status: draft
Date: 2026-08-15
Completed: 2026-08-16
Authority: 阶段 3 common 化执行计划（LAPJV / 雷达方程 / 天线方向图）
Related-Authority:
  - 评估：`common_consolidation_assessment_2026-08-15.md`
  - 迁移唯一状态：`remote_identification_radar_migration_status_2026-08-15.md`
  - 九项能力边界：`rir_signal_chain_capability_boundary_2026-08-15.md`
---

# AR/RIR common 化执行计划（阶段 3 #1-#3）

## 0. 目标

将 AR/RIR 之间的三组“同形副本”收敛到 `src/common/` 单源：

- LAPJV 全局最优指派求解器
- 雷达方程全集（回波/噪声/积累/误差/Pd/门限）
- 天线方向图四个模型（高斯/抛物线/余弦幂/sinc² + 扫描损失/旁瓣/后瓣）

两侧保留薄适配层，模块内类名/函数名不变，调用方零改动。

## 1. 收敛范围与落点

| # | 候选 | AR 位置 | RIR 位置 | 建议落点 |
|---|---|---|---|---|
| 1 | LAPJV 指派求解器 | `src/airborne_radar/signal/association/LapjvSolver.*` | `src/remote_identification_radar/tracking/RirLapjvSolver.*` | `src/common/optimization/LapjvSolver.{h,cpp}` |
| 2 | 雷达方程全集 | `src/airborne_radar/signal/detection/RadarEquations.*` | `src/remote_identification_radar/internal/RirRadarEquations.*` | `src/common/radar/RadarEquations.{h,cpp}` |
| 3 | 天线方向图 4 模型 | `src/airborne_radar/signal/detection/AntennaPatternRuntime.h` | `src/remote_identification_radar/dwell/RirAntennaPatternRuntime.h` | `src/common/radar/AntennaPatternRuntime.h`（header-only） |

## 2. 公共接口设计

### 2.1 LAPJV

- 命名空间：`oneq::common::optimization`
- 类型：`class LapjvSolver`
- 接口：`std::vector<int> Solve(const Eigen::Ref<const Eigen::MatrixXf>& cost_matrix) const`
- 行为：与现有 AR/RIR 实现完全一致；非方阵/空矩阵返回空解并记录错误日志。

### 2.2 雷达方程

- 命名空间：`oneq::common::radar`
- 类型：`struct RadarEquations`（全部 static）
- 通用 Swerling 枚举：`enum class SwerlingModel { kSwerling0, kSwerling1, kSwerling2, kSwerling3, kSwerling4 }`
- 函数签名全部改为**标量参数**，不引用任何模块 config：
  - `ComputeEchoPowerWithGain_dBW(float peak_power_w, float transmit_loss_db, float frequency_hz, float one_way_gain_db, float rcs_m2, float range_m, float propagation_loss_db)`
  - `ComputeEchoPower_dBW(float peak_power_w, float transmit_loss_db, float frequency_hz, float main_beam_gain_db, float rcs_m2, float range_m, float propagation_loss_db)`
  - `ComputeThermalNoisePower_W(float bandwidth_hz, float noise_figure_db)`
  - `ComputeIntegrationGain(int pulse_count)`
  - `ComputeRangeErrorStdDev(float snr_db, float bandwidth_hz)`
  - `ComputeAngleErrorStdDev(float snr_db, float beamwidth_rad)`
  - `ComputeDetectionProbability(float snr_db, float pfa, SwerlingModel model, int num_pulses)`
  - `ComputeThreshold(double pfa, int num_pulses)`
  - `MarcumQ(int order, double a, double b)`
  - `ThresholdDecision(float detection_prob, std::mt19937& rng)`

### 2.3 天线方向图

- 命名空间：`oneq::common::radar`
- 通用枚举：`enum class AntennaPatternModelType { kGaussianMainLobe, kParabolicMainLobe, kCosinePower, kSincPattern }`
- 通用中间结构：`BeamwidthDeg`、`LookOffsetDeg`、`PatternSample`
- 函数：
  - `IsInsideMainLobe(...)`
  - `ComputeMainLobeAttenuationDb(...)`
  - `ComputeScanLossDb(...)`
  - `EvaluateAntennaPattern(...)`
- 所有函数不引用模块 config；配置字段（模型类型、旁瓣/后瓣电平、扫描损失系数、波束指向）均以标量/简单结构传入。

## 3. 适配层改造

### 3.1 AR 侧

- `airborne_radar::signal::association::LapjvSolver` 保留，内部持有/调用 `oneq::common::optimization::LapjvSolver`。
- `airborne_radar::signal::detection::RadarEquations` 保留全部 static 函数，内部将 `config::engineering::*` 拆成标量后调用 common。
- `airborne_radar::signal::detection::AntennaPatternRuntime.h` 保留现有函数与中间类型，内部将 `config::engineering::AntennaPatternConfig` 等拆成 common 参数。

### 3.2 RIR 侧

- `remote_identification_radar::tracking::RirLapjvSolver` 保留，内部调用 common。
- `remote_identification_radar::internal::RirRadarEquations` 保留全部 static 函数，内部将 `config::hardware::*` 拆成标量后调用 common。
- `remote_identification_radar::dwell::RirAntennaPatternRuntime.h` 保留现有函数，内部调用 common。

## 4. CMake 与构建

1. 在 `src/common/CMakeLists.txt` 的 `ONEQ_COMMON_SOURCES` 中新增：
   - `optimization/LapjvSolver.cpp`
   - `radar/RadarEquations.cpp`
2. 确认 `oneq_common` 链接依赖已包含 `boost::math`（现有 `ONEQ_LINK_DEPENDENCIES` 已覆盖）。
3. 若新增目录，确保目录下有 `CMakeLists.txt` 不需要单独处理（由 `oneq_add_component` 统一 glob 源文件列表）。

## 5. 测试计划

1. 新增 common 单元测试：
   - `tests/unit/common/common_lapjv_solver_test.cpp`
   - `tests/unit/common/common_radar_equations_test.cpp`
   - `tests/unit/common/common_antenna_pattern_test.cpp`（如测试框架允许 header-only）
2. 保留并运行现有 AR/RIR 测试：
   - `ar_lapjv_solver_test`、`ar_signal_association_test`
   - `rir_track_associator_test`、`rir_radar_equations_test`、`rir_antenna_pattern_test`
   - 这些测试改为验证“薄适配层 + common 单源”后行为不变。
3. 等价回归门：
   - 用现有 AR/RIR 测试输入分别调用 common 与旧副本（如有 golden），要求逐位一致。
   - 如发现既存分叉，先登记再定基准，不静默取一侧。

## 6. 实施步骤

1. 新建 `src/common/optimization/LapjvSolver.{h,cpp}`、`src/common/radar/RadarEquations.{h,cpp}`、`src/common/radar/AntennaPatternRuntime.h`。
2. 更新 `src/common/CMakeLists.txt`。
3. 改造 AR 侧三个适配层。
4. 改造 RIR 侧三个适配层。
5. 新增/调整 common 与两侧测试。
6. 构建并运行：
   - `unit::airborne_radar`
   - `unit::remote_identification_radar`
   - `unit::common`
   - `integration::cross_domain`
   - `replay::remote_identification_radar`
   - `contract::public_api`
7. 更新 `algorithms.md` / `boundaries.md` 等文档，登记 common 单源位置。

## 7. 风险与核对项

- 噪声功率带宽口径：AR 检测单元用匹配滤波带宽，RIR 方程取发射 `bandwidth_hz`；common 化时以调用方传入带宽为准，不隐式选择。
- `ComputeEchoPowerWithGain_dBW` 的增益叠加语义：两侧需显式对账。
- 方向图主瓣判定边界、旁瓣/后瓣电平闭合条件：common 化时以现有两侧共同行为为基准。
- 日志前缀差异：common 统一使用 `[LapjvSolver]` / `[RadarEquations]` 等中性前缀，两侧不再各自打印。
- Swerling 枚举、方向图模型枚举的映射：AR/RIR 适配层负责转换，common 只使用通用枚举。

## 8. 验收标准

1. AR/RIR 对同一物理输入的输出与收敛前逐位一致（等价回归绿）。
2. `src/common/` 实现无任何模块 config 类型依赖。
3. AR/RIR include 闭包互不引用（不变式保持）。
4. 全部相关单元/集成/契约测试通过。

## 9. 完成记录

- LAPJV：`src/common/optimization/LapjvSolver.{h,cpp}` 已创建；AR/RIR 改为薄适配层。
- 雷达方程：`src/common/radar/RadarEquations.{h,cpp}` 已创建；AR/RIR 改为薄适配层。
- 天线方向图：`src/common/radar/AntennaPatternRuntime.h` 已创建；AR/RIR 改为薄适配层。
- 已通过验证：
  - `unit::common` 138/138
  - `unit::airborne_radar` 585/585
  - `unit::remote_identification_radar` 115/115
  - `integration::remote_identification_radar` 29/29
  - `replay::remote_identification_radar` 3/3
  - `integration::cross_domain` 6/6
  - `contract::public_api` 7/7
