# SBIRS 状态估计滤波（EKF/CKF）接入方案

Status: draft
Authority: 调研与详细设计草案；本文不构成当前设计权威，结论落定后回写 `docs/space_based_infrared_sensor/design.md`

> **实施进展（2026-07-08）**：本文 §4.2 的**选项 B（抽到 `src/common/estimation`）已实施**。
> 滤波原语（KF/EKF/UDKF/SRIF/IMM）已从 `airborne_radar::signal::tracking` 迁至
> `oneq::common::estimation` 并维度模板化为 `GaussianState<StateDim, MeasDim>`；
> airborne_radar 通过 `signal/tracking/` 下的薄外观头（re-export `using` 别名，6/3 实例化）
> 零回归消费 common 层（1489 单元测试全通过）。
> 因此 §4.2 的选项 A/B/C 抉择已尘埃落定；§5 起的 SBIRS 接入步骤（状态拆分、EKF 量测模型、
> snapshot/配置扩展）仍是后续阶段，尚未实施。

本文调查 SBIRS 模块从"真值辅助跟踪"升级到"EKF/CKF 状态估计滤波"的可行路径、前置条件、接入点
和分阶段落地建议。调研覆盖 `airborne_radar` 已有滤波框架（可复用资产）、SBIRS 当前架构（接入点）、
两者的对接约束（维度、命名空间、snapshot/replay、配置、输出），以及 EKF 与 CKF 的选型差异。

设计权威 `docs/space_based_infrared_sensor/design.md`（以下简称 design.md）当前将 EKF/CKF 明确列为
**非目标**（design.md:718-720），并在 `TruthAssistedTracking` 适用边界中要求"后续若引入估计滤波，
**必须先把本状态重命名或拆分**"（design.md:453）。本文即围绕此前置展开。

> 行号说明：本文引用的 design.md、源码行号基于 `codex/sbirs-sensor-dev` 分支截至 2026-07-08 的状态。
> design.md 中对 `docs/review/sbirs_filter.md` 的行号引用（如 `:886-994`、`:1190-1216`）目前已悬空——
> 该历史草案已裁剪至 134 行，EKF/CKF 公式章节已删除，仅保留摘要与一句"后续可扩展……EKF 跟踪，
> 但第一版不实现"（`docs/review/sbirs_filter.md:91`）。本文不再依赖该草案的公式行号。

---

## 0. TL;DR

1. **仓库里已有成熟滤波框架，但在 `airborne_radar` 模块下，SBIRS 自己是零。** `airborne_radar/signal/tracking`
   提供了 KF/EKF/UD-KF/SRIF 四后端 + IMM，预测器/更新器分离 + 接口注入，设计干净。SBIRS 无任何协方差代码。
2. **不能直接照抄过来。** 三个硬前置：(a) 拆分 `TruthAssistedTracking` 状态（design.md:453 强制要求）；
   (b) 决定滤波框架的模块归属（当前是 `airborne_radar::signal::tracking`，SBIRS 不能跨模块依赖它的 internal 头）；
   (c) 处理状态/量测维度——框架硬编码 6 维状态 / 3 维笛卡尔量测，SBIRS 是角度被动量测。
3. **EKF 接入成本最低，CKF 需新写但接口契合。** EKF 的 `IMeasurementModel`/`ITransitionModel` 扩展点
   正好支持 SBIRS 的非线性角度量测；CKF 是 sigma-point 路径，没有现成实现，但可以套同样的
   predictor/updater 接口契约。
4. **对接入面的影响是可控的**：snapshot 必须扩展（否则回滚/replay 破坏滤波连续性），配置需新增 tracking 子域，
   raw output 不变，replay 需要纳入滤波随机源 seed。controller 的 capture/restore 机制无需改动，自动覆盖滤波器。

---

## 1. 现状基线

### 1.1 SBIRS 当前跟踪实现（真值辅助，无滤波）

SBIRS 的"跟踪"是 design.md:2.5 描述的**真值辅助**：首次捕获成功后，pipeline 每周期直接用输入场景的
目标真值位置重算 LOS、方位/俯仰、距离、SNR，生成检测输出。不维护任何估计状态、协方差或滤波器。

关键事实（源码核对）：

- **状态机不是独立类**，而是 `SbirsPipeline` 内联的 `std::map<std::uint64_t, SbirsTargetState>` 枚举表
  （`src/sbirs_sensor/pipeline/SbirsPipeline.h:93`）。5 状态枚举定义在 `SbirsPipeline.h:22-28`：
  `kUndetected / kWideCandidate / kAwaitingNfovAcquisition / kTruthAssistedTracking / kLost`。
- **每个 target 只存一个枚举值**——`target_states_` 是 `std::map<target_id, SbirsTargetState>`，
  没有任何位置、速度、协方差字段（`SbirsPipeline.h:90-96`）。
- **`TruthAssistedTracking` 阶段不依赖历史状态**：track 分支（`SbirsPipeline.cpp:139-154`）每帧从
  `target.position_ecef_m` 实时重算 az/el/range/snr，所谓"指向"就是真值方位/俯仰，没有指向控制器。
- **cue 延迟线性外推**（`SbirsPipeline.cpp:179-193`）：`predicted_position = position + velocity * cue_latency_s`，
  这是 EKF predict 步的退化（确定性线性）版本。
- **snapshot 只保存枚举 + 随机源**（`SbirsPipeline.h:34-41`）：
  ```cpp
  struct SbirsPipelineSnapshot {
    float scan_azimuth_deg;
    std::uint64_t next_detection_id;
    std::map<std::uint64_t, SbirsTargetState> target_states;  // 仅枚举
    bool has_locked_target;
    std::uint64_t locked_target_id;
    unsigned int random_state;  // 误差模型随机源
  };
  ```
- **replay 不持久化 snapshot**：replay session 通过重放 config + 逐周期 input 并比对 `SbirsCycleResult`
  实现复现（`src/sbirs_sensor/session/SbirsReplaySession.cpp`），snapshot 只用于 controller 内部回滚
  （`src/sbirs_sensor/runtime/SbirsController.cpp:29,44-48`）。

### 1.2 仓库已有滤波资产（airborne_radar）

`airborne_radar::signal::tracking`（全部 internal，头在 `src/airborne_radar/signal/tracking/`，未 install
到 `include/1q/`）提供了完整的 Kalman 滤波家族：

| 后端 | Predictor | Updater | 备注 |
|---|---|---|---|
| 标准 KF（Joseph 形式） | `KalmanPredictor.h:29` | `KalmanUpdater.h:30` | 基线 |
| EKF | `EkfFilter.h:125` | `EkfFilter.h:158` | 非线性 model 注入 |
| UD-KF | `UdkfPredictor.h:20` | `UdkfUpdater.h:18` | 数值稳定化 |
| SRIF | `SrifPredictor.h:20` | `SrifUpdater.h:18` | 信息形式 |
| IMM | `ImmFilter.h:51` | — | 多模型组合器 |

**架构参照（不只是滤波器本身）**：airborne_radar 还提供了 SBIRS 未来接入时应参照的"上层"设施：
`TrackLifecycleManager`（`src/airborne_radar/signal/tracking/TrackLifecycleManager.h`，含
`CaptureRuntimeState`/`RestoreRuntimeState`）、`DataAssociation`（关联）、`Hypothesiser`（假设生成）、
`SignalComponentFactory`（后端工厂）、`RuntimeAssemblySupport`（可注入装配）。SBIRS 当前是
"无 tracking/无 association 抽象、状态机内联、单目标锁定"的最简形态。

---

## 2. 可复用框架的精确接口

本节是 SBIRS 接入时可直接对照的接口契约。全部位于 `airborne_radar::signal::tracking` 命名空间。

### 2.1 数据载体 GaussianTrackState

`src/airborne_radar/signal/tracking/GaussianTrackState.h`：

```cpp
// 维度：硬编码 static constexpr，不是模板参数（GaussianTrackState.h:18-23）
static constexpr int kStateDim = 6;        // [x, vx, y, vy, z, vz]
static constexpr int kMeasurementDim = 3;  // [x, y, z]

// 类型别名：全部 Eigen 固定维度 float 矩阵（栈分配）
using StateVector            = Eigen::Matrix<float, 6, 1>;   // :27
using StateCovariance        = Eigen::Matrix<float, 6, 6>;   // :31
using MeasurementVector      = Eigen::Matrix<float, 3, 1>;   // :35
using MeasurementCovariance  = Eigen::Matrix<float, 3, 3>;   // :39
using TransitionMatrix       = Eigen::Matrix<float, 6, 6>;   // :43
using MeasurementMatrix      = Eigen::Matrix<float, 3, 6>;   // :51
using KalmanGainMatrix       = Eigen::Matrix<float, 6, 3>;   // :55

struct GaussianTrackState {                                  // :61-72
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  StateVector mean{StateVector::Zero()};
  StateCovariance covariance{StateCovariance::Identity()};
};
```

**维度泛化是接入的最大结构性障碍**（见 §4.3）。6 维 `[x,vx,y,vy,z,vz]` 与 SBIRS 的红外被动角度量测
不完全匹配；硬编码点散布在 `BuildPositionMeasurementMatrix()`（`IKalmanUpdater.h:65-71`，`H(0,0)/H(1,2)/H(2,4)`）、
`BuildTransitionMatrix`（`KalmanPredictor.cpp:26-30`）等处。

### 2.2 抽象接口

`IKalmanPredictor`（`IKalmanPredictor.h:28-45`）：
```cpp
struct KalmanPredictorConfig { float noise_diff_coeff{1.0f}; };  // 过程噪声扩散系数 q
class IKalmanPredictor {
 public:
  virtual GaussianTrackState Predict(const GaussianTrackState& prior, float dt) const = 0;
  virtual void UpdateConfig(KalmanPredictorConfig) {}  // 运行期调参，默认空
};
```

`IKalmanUpdater`（`IKalmanUpdater.h:33-60`）：
```cpp
struct KalmanUpdaterConfig { float measurement_noise_std{10.0f}; };
struct KalmanUpdateResult {
  GaussianTrackState posterior;
  MeasurementVector innovation;              // y = z - H·x̂
  MeasurementCovariance innovation_covariance;  // S = H·P̂·Hᵀ + R
};
class IKalmanUpdater {
 public:
  // 静态 R（用 config 的 std 构造）
  virtual KalmanUpdateResult Update(const GaussianTrackState& predicted,
                                    const MeasurementVector& measurement) const = 0;
  // 动态 R（逐测量注入，SBIRS 的角度/距离协方差随距离变化，必须用此重载）
  virtual KalmanUpdateResult Update(const GaussianTrackState& predicted,
                                    const MeasurementVector& measurement,
                                    const MeasurementCovariance& dynamic_R) const = 0;
  // 共享工具
  static MeasurementMatrix BuildPositionMeasurementMatrix();      // 位置提取 H
  static MeasurementCovariance BuildDefaultMeasurementNoise(float std_dev);
};
```

**动态 R 重载对 SBIRS 至关重要**：SBIRS 的测量噪声随距离、折射、动态滞后变化（见 §5.4），不能用静态 R。
airborne_radar 生产路径已全程走 3 参数重载（`TrackLifecycleManager.cpp:652-654`，R 来自
`measurement.raw_measurement.measurement_covariance`）。

### 2.3 EKF 扩展点（SBIRS 最可能的先期后端）

`EkfFilter.h` 定义了非线性 model 接口，这是接入 SBIRS 角度量测的关键：

```cpp
class ITransitionModel {                    // EkfFilter.h:22-39
  virtual StateVector Function(const StateVector& state, float dt) const = 0;   // f(x,dt)
  virtual TransitionMatrix Jacobian(const StateVector& state, float dt) const = 0;  // F
};
class IMeasurementModel {                   // EkfFilter.h:44-59
  virtual MeasurementVector Function(const StateVector& state) const = 0;   // h(x)
  virtual MeasurementMatrix Jacobian(const StateVector& state) const = 0;   // H
};
```

EKF 构造为**非拥有裸指针注入**（model 由调用方保活）：
```cpp
EkfPredictor(const ITransitionModel* model, EkfPredictorConfig = {});   // EkfFilter.h:132
EkfUpdater(const IMeasurementModel* model, EkfUpdaterConfig = {});      // EkfFilter.h:165
```

数值稳定性技巧（`EkfFilter.cpp`）：预测协方差 `P̂ = F·P·Fᵀ + Q`（线性化），更新用 **Joseph 形式**
后验协方差 `(I-KH)P̂(I-KH)ᵀ + KRKᵀ`，增益用 `Eigen::LLT`（Cholesky）解新息协方差避免显式求逆，
LLT 分解失败时跳过更新并回退为预测态（`EkfFilter.cpp:56-64`）。

**接入方式**：SBIRS 只需实现一个自定义 `IMeasurementModel`（h(x) = 笛卡尔状态→球坐标角度，H 为其 Jacobian），
即可 `new EkfUpdater(&sbirs_meas_model, config)`，无需改 framework。工厂默认注入的是线性 model
（`SignalComponentFactory.cpp:208` 的 `static const LinearPositionMeasurementModel`），非线性必须自行构造。

---

## 3. SBIRS 接入点清单

按改动层次从内到外，共 12 个精确接入点。**前 3 个是结构性的（必须先做），其余是接线性的。**

### 3.1 结构性接入点（前置）

| # | 位置 | 现状 | 滤波器接入需要的改动 |
|---|---|---|---|
| **A** | `SbirsPipeline.h:93` `target_states_` | `map<id, SbirsTargetState>`（仅枚举） | 扩成 `map<id, TargetTrackState>`，TrackState 含状态枚举 + 滤波状态（`GaussianTrackState` 或等价）+ 协方差 |
| **B** | `SbirsPipeline.h:34-41` `SbirsPipelineSnapshot` | 仅枚举 + random_state | **必须**加滤波状态（均值向量 + 协方差矩阵 + 滤波随机源），否则 controller 回滚（`SbirsController.cpp:44-48`）和 replay 复现都会破坏滤波连续性 |
| **C** | design.md:2.2 状态机 + `SbirsPipeline.h:22-28` 枚举 | `kTruthAssistedTracking` 一个状态混用真值辅助与（未来的）测量跟踪 | 拆分为 `kTruthAssistedTracking`（保留真值辅助）与 `kEstimatedTracking`（滤波测量跟踪）两态，满足 design.md:453 强制前置 |

### 3.2 算法接入点（pipeline 内部）

| # | 位置 | 现状 | 改动 |
|---|---|---|---|
| 1 | `SbirsPipeline.cpp:139-154`（track 分支） | 每帧用真值 az/el 重算输出 | `kEstimatedTracking` 态下改为：滤波器 predict → 用本帧带误差测量 update → 输出滤波估计（可叠加显示噪声） |
| 2 | `SbirsPipeline.cpp:179-193`（cue 外推） | `position + velocity * latency` 线性外推 | 退化为滤波器 predict 的特殊情况；或保留作为首次捕获的几何外推（不进滤波器） |
| 3 | `SbirsPipeline.cpp:199-254`（NFOV 首次捕获） | cue 指向用带误差 measured az/el；窗口判定用真值 az/el | 捕获成功时**初始化**滤波状态（用首次测量 + 初始协方差），转入 `kEstimatedTracking` |
| 4 | `SbirsPipeline.cpp:173-178`（误差施加） | `ApplyAngularErrorModel` 生成 measured | 测量值与滤波 update 的测量方程对齐；R 矩阵从误差模型 sigma 显式构造（见 §5.4） |

### 3.3 配置/装配接入点

| # | 位置 | 改动 |
|---|---|---|
| 5 | `include/1q/sbirs_sensor/config/SbirsPolicyConfig.h:50-54` | 新增 `SbirsTrackingConfig`（过程噪声 q、初始协方差 P0、滤波后端选择、滤波随机源 seed）。当前**无任何 tracking/filter 配置位** |
| 6 | `include/1q/sbirs_sensor/config/SbirsRuntimeConfigPatch.h:22-40` | 新增 tracking 运行期可调位（注意：当前 policy 是整域覆盖，滤波参数需独立 `has_tracking` 位，否则改 R 参数会整域重置 policy） |
| 7 | `src/sbirs_sensor/runtime/SbirsPipelineConfigMapper.cpp:6` `MapSessionToInternal` | 从 session config 解析滤波器配置（目前是直通 `internal.session = config`） |
| 8 | `src/sbirs_sensor/session/SbirsSessionCompositionRoot.cpp:8-12` `CreateSbirsController` | **滤波器工厂注入点**。当前极简（直接 `new SbirsController(MapSessionToInternal(config))`）。需在此构造 predictor/updater（或参照 airborne_radar 的 `RuntimeAssemblySupport` 可注入装配） |

### 3.4 输出/replay 接入点

| # | 位置 | 改动 |
|---|---|---|
| 9 | `SbirsOutputFrame` / `SbirsDetectionRecord`（`SbirsOutputTypes.h:29-36`） | **不变**。raw output 仍是角度/SNR/observation_stage/detected。三层分离原则禁止把滤波估计/协方差塞进 raw output（design.md:696-699） |
| 10 | `SbirsDetectionAttributionRecord`（`SbirsOutputTypes.h:60-68`） | 若需暴露滤波估计/新息/协方差供诊断，加字段或新增 `SbirsTrackingAttributionRecord`（进 result/debug 层，不进 raw） |
| 11 | `schemas/replay/sbirs_replay.fbs` / `sbirs_session_replay.fbs` | 滤波随机源 seed 必须进 replay（否则过程噪声采样不可复现）；滤波状态是否进 replay 取决于是否走 snapshot 持久化（当前不走，靠重放复现） |
| 12 | `SbirsPipeline.cpp:75-77` `ApplyConfig` | runtime patch 时的滤波器重置策略：建议只改 R/Q 参数（`UpdateConfig`），不重置协方差/状态向量（否则破坏跟踪连续性） |

---

## 4. 三个硬前置的详细论证

### 4.1 前置一：拆分 TruthAssistedTracking（design.md 强制）

design.md:453 原文："后续若引入估计滤波，**必须先把本状态重命名或拆分**，避免把真值辅助和真实测量跟踪混用。"

**为什么不能在现有 `kTruthAssistedTracking` 上直接挂滤波器**：该状态当前的语义是"用仿真真值生成 NFOV 指向"
（design.md:2.5，`SbirsPipeline.cpp:139-154`）。如果在同一状态里混入滤波估计，会出现两种根本不同的跟踪来源
（真值 vs 测量估计）共用一个状态枚举，导致：
- 输出语义不可追溯（某帧的检测记录到底是真值还是估计？）；
- capture/restore 无法正确回滚（真值辅助无状态可回滚，滤波器有）；
- replay 语义模糊（真值辅助确定性复现，滤波器依赖随机源）。

**建议拆分方案**（6 状态）：

| 状态 | 含义 | 指向来源 |
|---|---|---|
| `kUndetected` | （不变） | — |
| `kWideCandidate` | （不变） | WFOV 带误差 |
| `kAwaitingNfovAcquisition` | （不变） | WFOV cue |
| `kTruthAssistedTracking` | **保留**，仿真稳定性假设，默认路径 | 真值 |
| `kEstimatedTracking`（新） | 滤波测量跟踪，可选启用 | 滤波器估计 |
| `kLost` | （不变） | — |

新增 `kEstimatedTracking` 由配置开关控制是否进入（见 §5.1）。不开启时行为完全等同现状，向后兼容。
状态转移：`kAwaitingNfovAcquisition → kTruthAssistedTracking`（默认）或 `→ kEstimatedTracking`（启用滤波），
由配置决定。两种跟踪态的退出条件一致（目标消失/传感器关闭 → `kLost`）。

**对 design.md 的影响**：需同步 design.md:2.2 状态转移图、转移条件表（design.md:346-353）、2.5 适用边界，
以及 `sbirs_state_machine_test`、`sbirs_pipeline_test`。这属于 design.md:4 设计变更规则第 2 项（状态机变化）。

### 4.2 前置二：滤波框架的模块归属

**问题**：滤波框架当前在 `airborne_radar::signal::tracking`（internal 头，`src/airborne_radar/signal/tracking/`）。
SBIRS 不能跨模块依赖 airborne_radar 的 internal 头——这违反模块独立性（design.md:62-63 "SBIRS 不在公开头文件
中暴露 electro_optical_sensor"的精神同理适用于不依赖 airborne_radar internal）。

**三个归属选项**：

| 选项 | 做法 | 优点 | 缺点 |
|---|---|---|---|
| **A. 复制到 SBIRS 内部**（参照 EOS foundation 模式） | 把 tracking 子域复制到 `src/sbirs_sensor/tracking/`，改命名空间为 `sbirs_sensor::tracking`，按 SBIRS 场景调整维度/量测模型 | 与 design.md:1.3 "foundation 参照 EOS 复制改名"模式完全一致；模块独立；不引入跨模块依赖 | 代码重复（与 airborne_radar 各存一份）；后续滤波算法演进需双向同步 |
| **B. 抽到 common 公共层** | 把 tracking 子域提升到 `src/common/estimation/`，命名空间 `oneq::common::estimation`，airborne_radar 与 sbirs 都消费 | 无重复；契约层（`contract.md:46-61`）明确支持 `src/common/` 跨模块复用 | 改动面最大（要重构 airborne_radar 现有消费）；需泛化维度（§4.3）；airborne_radar 现有测试/装配需迁移 |
| **C. SBIRS 内部独立实现** | 不复用，SBIRS 从零写自己的 EKF/CKF | 完全自主，可按 SBIRS 角度量测场景量身设计 | 放弃已有成熟实现；airborne_radar 的数值稳定性技巧（Joseph、LLT）要重写；最不符合"DRY" |

**建议**：短期选 **A**（与 design.md 已确立的 foundation 复制模式一致，风险最低，不触碰 airborne_radar）；
若后续 EOS/ESR/SAR 也需要滤波估计，再启动 **B**（抽到 common）。**B 是长期正确方向**，但应在至少两个模块
有真实滤波需求时再做，避免过早抽象。本文按 **A** 撰写接入细节，§6.2 给出 **B** 的迁移触发条件。

**选项 A 的复制范围**（最小集，满足 EKF 接入）：
`GaussianTrackState.h`、`IKalmanPredictor.h`、`IKalmanUpdater.h`、`EkfFilter.{h,cpp}`、
`KalmanPredictor.{h,cpp}`（复用其 `BuildTransitionMatrix`/`BuildProcessNoise`）。UD-KF/SRIF/IMM 首期不复制
（SBIRS 单目标锁定，不需要 IMM；数值稳定性问题首期不突出）。

### 4.3 前置三：状态/量测维度

**框架现状**：`kStateDim=6`（`[x,vx,y,vy,z,vz]`）、`kMeasurementDim=3`（`[x,y,z]` 笛卡尔），硬编码
`static constexpr`，非模板参数（`GaussianTrackState.h:18-23`）。

**SBIRS 场景**：
- **状态**：仍可用 6 维 CV（`[x,vx,y,vy,z,vz]` ECEF），因为目标运动学是笛卡尔的。这点与 airborne_radar 兼容。
- **量测**：红外被动量测是**球坐标角度** `[az, el]`（2 维）或 `[az, el, range]`（3 维，但 range 是带误差的弱观测，
  design.md:652-653 明确 range 不进 raw output）。量测方程 h(x) = 笛卡尔→球坐标是**非线性**的——这正是要用 EKF 的原因。

**维度对接方案**：

- **方案 1（量测维保持 3）**：量测向量定义为 `[az, el, range]`，实现非线性 `IMeasurementModel`：
  `h(x) = [atan2(y,x), asin(z/r), sqrt(x²+y²+z²)]`，Jacobian 解析或数值求导。range 通道的 R 设大
  （反映被动测距不确定性），或直接置 `range` 通道 R 为极大值（近似不观测 range）。**优点**：维度与框架一致，
  复用 `MeasurementVector=Matrix<float,3,1>`。**缺点**：把不可观测的 range 塞进量测，需要用 R 屏蔽。
- **方案 2（量测维改 2）**：修改 `kMeasurementDim=2`（纯角度量测 `[az, el]`）。**优点**：语义干净。
  **缺点**：要改 `GaussianTrackState.h` 的 `kMeasurementDim` 与所有派生类型别名，影响 `BuildPositionMeasurementMatrix`
  等硬编码点；若走选项 A（复制到 SBIRS），这些改动隔离在 SBIRS 内部，不污染 airborne_radar。

**建议**：选项 A（复制）下选**方案 2**（量测维 2），在 SBIRS 自己的 `GaussianTrackState.h` 里设
`kMeasurementDim=2`，重写 `IMeasurementModel` 为纯角度。这是最干净的语义。若后续走选项 B（抽到 common），
则需把维度模板化（`template<int StateDim, int MeasDim>`），改动更大但一次到位。

---

## 5. EKF/CKF 选型与关键设计

### 5.1 配置开关与后端选择

新增 `SbirsTrackingConfig`（挂 `SbirsPolicyConfig`）：

```cpp
struct SbirsTrackingConfig {
  bool enable_estimated_tracking{false};   // 默认 false，保持真值辅助现状（向后兼容）
  TrackingBackend backend{TrackingBackend::kEkf};  // kEkf / kCkf（首期只实现 kEkf）
  float process_noise_diff_coeff{1.0f};    // 过程噪声 q（CV 模型扩散系数）
  float initial_position_std_m{1000.0f};   // 初始位置 1-σ，构造 P0
  float initial_velocity_std_m_per_s{25.0f}; // 初始速度 1-σ
  unsigned int filter_random_seed{0U};     // 滤波随机源 seed；0 表示复用 error_model.random_seed
};
```

`enable_estimated_tracking=false`（默认）时，状态机不会进入 `kEstimatedTracking`，行为与现状逐字节一致。
这是向后兼容的硬保证，也满足 design.md 非目标边界的渐进式放宽。

### 5.2 EKF 量测模型（SBIRS 专用）

SBIRS 的非线性量测模型（状态 ECEF 笛卡尔 → 量测球坐标角度），需实现 `IMeasurementModel`：

- `h(x)`：`az = atan2(y, x)`，`el = asin(z / r)`（`r = sqrt(x²+y²+z²)`，参照 design.md:547 的 EOS az/el 公式）
- `H = ∂h/∂x`（2×6 Jacobian），解析表达式或数值差分。注意 atan2/asin 在极点附近的奇异性需处理。

此 model 是 SBIRS 接入的核心新增代码。实现后 `new EkfUpdater(&sbirs_angle_measurement_model, config)` 即可。

### 5.3 CKF（容积卡尔曼）评估

**现状**：仓库**无任何 CKF 实现**。airborne_radar 只有 EKF（Jacobian 路径），没有 sigma-point 路径。

**算法差异**：

| 维度 | EKF | CKF |
|---|---|---|
| 非线性传播 | Jacobian 线性化（一阶截断） | 3rd-order spherical-radial cubature 点（无需 Jacobian） |
| 精度 | 强非线性下偏差大 | 强非线性下优于 EKF |
| Jacobian 需求 | 需要解析/数值 Jacobian | 不需要（只用函数求值） |
| 接口契合 | 现有 `ITransitionModel`/`IMeasurementModel` 的 `Function` 可直接用，`Jacobian` 空实现 | 同上，但 predictor/updater 内部要新增 sigma-point 传播逻辑 |
| 实现成本 | 低（框架已有，只写 model） | 中（要新写 `CkfPredictor`/`CkfUpdater`，实现 sigma-point 生成与权重） |

**接口复用可行性**：CKF 可以套同样的 `IKalmanPredictor`/`IKalmanUpdater` 接口——predictor/updater 内部
用 sigma-point 传播 `GaussianTrackState`，对外仍是 `Predict(prior, dt)` / `Update(predicted, meas[, R])`。
model 接口可以**复用** `ITransitionModel`/`IMeasurementModel` 的 `Function`（CKF 不用 `Jacobian`），
或新增 `INonlinearModel`（只含 `Function`）由 CKF 消费。

**建议**：首期只做 **EKF**。理由：(a) airborne_radar 已有成熟 EKF 可复制参照；(b) SBIRS 红外角度量测的
非线性度中等（atan2/asin 在正常观测几何下不极端），EKF 精度通常足够；(c) CKF 需新写 sigma-point 传播，
是独立工作量。若后续发现 EKF 在大俯仰角或近距离机动场景下偏差显著，再增加 CKF 后端——届时接口已就位，
只需新增 `CkfPredictor`/`CkfUpdater` 两个类 + 配置枚举值。design.md:718 把 EKF/CKF 并列，两者都可在此方案下分阶段落地。

### 5.4 R 矩阵（测量噪声协方差）来源

SBIRS 已有的 `SbirsErrorModel`（`src/sbirs_sensor/foundation/SbirsErrorModel.h`）是 R 矩阵的天然素材，
但目前只用于给输出加噪（`ApplyAngularErrorModel`），**没有显式产出 R 矩阵的函数**——需新增。

R 矩阵（2×2，`[az, el]`）的对角元应取自 `SbirsErrorModelConfig`（`SbirsPolicyConfig.h:28-36`）：
- `σ_az² = σ_orbit² + σ_attitude² + σ_fov² + σ_refraction(range,el)² + σ_lag(ω,bandwidth)²`（RSS 合成，单位 deg²）
- `σ_el²` 同上（折射/滞后项按俯仰通道单独算）
- `SbirsRandomSource::NextStandardNormal()` 的 Box-Muller 采样源目前驱动 `ApplyAngularErrorModel`，
  滤波器若需要过程噪声采样，应复用同种可 Capture/Restore 的源（保证 replay 一致性）。

**对接**：新增一个 `BuildMeasurementCovariance(error_model_config, range, elevation, angular_rate)` 返回
`MeasurementCovariance`（2×2），在 pipeline track 分支调用 3 参数 `Update(predicted, meas, dynamic_R)`。
这与 airborne_radar 的动态 R 路径（`TrackLifecycleManager.cpp:652-654`，R 来自每测量的协方差字段）模式一致。

---

## 6. 分阶段落地建议

### 阶段 0：状态拆分（前置，不含滤波器）

- 拆 `kTruthAssistedTracking` → 增加 `kEstimatedTracking`（§4.1），但 `kEstimatedTracking` 暂不启用
  （配置 `enable_estimated_tracking=false`）。
- 同步 design.md:2.2 状态机、转移条件表，更新 `sbirs_state_machine_test`。
- 扩展 `SbirsPipelineSnapshot` 增加 track state 字段（先放空的占位结构体，后续填滤波状态）。
- **此阶段产出可独立验证**：状态机 6 状态、snapshot 扩展、replay 兼容，不引入任何滤波代码。

### 阶段 1：EKF 接入（最小可用）

- 复制 tracking 最小集到 `src/sbirs_sensor/tracking/`（选项 A，§4.2），设 `kMeasurementDim=2`。
- 实现 SBIRS 角度量测 `IMeasurementModel`（§5.2）与 CV `ITransitionModel`。
- 新增 `BuildMeasurementCovariance`（§5.4），R 从 `SbirsErrorModelConfig` 构造。
- 在 `kEstimatedTracking` 态接 predict/update（接入点 #1），首次捕获初始化滤波状态（接入点 #3）。
- 新增 `SbirsTrackingConfig`（§5.1），composition root 注入滤波器（接入点 #8）。
- snapshot 填入真实滤波状态（接入点 #B）。
- 测试：`sbirs_tracking_estimation_test`（参照 `ar_kalman_filter_test.cpp` 的构造方式），覆盖 predict-update
  收敛、动态 R、capture/restore 一致性、replay 复现。

### 阶段 2（可选）：CKF 后端 + 关联

- 新增 `CkfPredictor`/`CkfUpdater`（sigma-point），套 `IKalmanPredictor`/`IKalmanUpdater` 接口。
- 配置增加 `kCkf` 后端枚举。
- 若需要多测量-航迹关联，参照 airborne_radar 的 `DataAssociation`/`Hypothesiser`（`FullMahalanobisDistanceMetric`
  用新息协方差做波门，design.md:718 提及的"波门关联"）。

### 迁移触发条件（何时从选项 A 升级到选项 B）

当满足以下任一条件，应启动"抽到 `src/common/estimation/`"的重构：
1. EOS/ESR/SAR 任一模块也产生真实的状态估计滤波需求；
2. SBIRS 与 airborne_radar 的滤波实现出现实质性算法分叉（维度模板化势在必行）；
3. 滤波框架的 bugfix/数值优化需要双向同步超过 2 次。

---

## 7. 对现有契约的影响核查

| 契约项 | 影响 | 说明 |
|---|---|---|
| design.md:718-720 EKF/CKF 非目标 | **放宽** | 需在 design.md 非目标条目注明"已由 `enable_estimated_tracking` 渐进开放"，并引用本文 |
| design.md:453 状态拆分前置 | **执行** | 阶段 0 即完成 |
| design.md:2.2 状态机 | **扩展** | 5 → 6 状态，按 design.md:4 变更规则 #2 同步转移图/条件表/测试 |
| design.md:2.5 真值辅助边界 | **保留** | 真值辅助仍是默认路径（`enable_estimated_tracking=false`），边界不变 |
| `contract.md:114-134` 运行期配置提交策略 | 不变 | SBIRS 仍是"立即提交"类；滤波状态进 snapshot 后，controller 内部 capture/restore 自动覆盖 |
| `contract.md:135-154` 三层输出模型 | 不变 | raw output 不含滤波估计；估计/协方差只进 result/debug 层（接入点 #10） |
| `contract.md:37-43` public API 边界 | 不变 | 滤波器是 internal（选项 A 下在 `src/sbirs_sensor/tracking/`），不暴露；只新增 `SbirsTrackingConfig` public 配置 DTO |
| replay 一致性 | **需扩展** | 滤波随机源 seed 进 replay schema（接入点 #11）；snapshot 不持久化（靠重放复现），但随机源状态必须在 snapshot 中 |

---

## 8. 待决问题（需进一步确认）

1. **量测维 2 vs 3**（§4.3 方案 1/2）：range 通道是用大 R 屏蔽（方案 1，维度不动）还是改为 2 维纯角度
   （方案 2，需改维度）？倾向于方案 2，但需确认 SBIRS 是否有任何场景需要 range 作为弱观测进滤波器。
2. **滤波随机源与误差随机源的关系**（§5.4）：滤波器过程噪声采样是复用 `SbirsRandomSource`（与误差模型共享，
   capture/restore 一致）还是独立源？复用更简单，但会让误差采样与滤波采样耦合在同一随机序列上。
3. **runtime patch 对滤波器的重置策略**（接入点 #12）：`ApplyConfig` 时是否重置协方差？建议只调参不重置状态，
   但需确认这与"立即提交"契约（`contract.md:130`）不冲突。
4. **选项 A 的复制是否触发 design.md:1.3 的"参照复制"记录义务**：design.md:1.3 记录了 foundation 从 EOS 复制
   改名的先例；tracking 从 airborne_radar 复制改名是否需要在 design.md 增加类似段落？

---

## 附录 A：airborne_radar Capture/Restore 参照实现

`TrackLifecycleManager` 的快照机制是 SBIRS snapshot 扩展的直接参照
（`src/airborne_radar/signal/tracking/TrackLifecycleManager.cpp:170-243`）：

- 快照用 `std::shared_ptr<void> opaque` 持有（`ITrackLifecycleManager.h:25`），带 `schema_version` 和
  `owner_identity` 校验。
- `CaptureRuntimeState`（`:170-194`）：值拷贝整个 `TrackState`（含内嵌 `GaussianTrackState` 的 Eigen 矩阵）进快照——
  协方差矩阵随结构体值拷贝带过，无单独序列化逻辑。
- `RestoreRuntimeState`（`:197-243`）：校验 schema_version + owner_identity，清空当前表，从池中重新 Acquire 并赋值。
- predictor/updater 是**非拥有裸指针**（`TrackLifecycleManager.h:295-296`），不进快照（它们是无状态策略对象，
  由工厂/装配层保活）。

SBIRS 可简化：snapshot 用值类型（`SbirsPipelineSnapshot` 已是值结构体），Eigen 矩阵直接作为字段值拷贝，
不需要 `shared_ptr<void>` 或池化。predictor/updater 同样作为非拥有指针由 composition root 保活。

---

*本文为调研草案。结论落定后，前置拆分与 EKF 接入细节应回写 `docs/space_based_infrared_sensor/design.md`，
框架归属决策（选项 A/B/C）若涉及跨模块应回写 `docs/common/contract.md` 或 `docs/common/open_questions.md`。*
