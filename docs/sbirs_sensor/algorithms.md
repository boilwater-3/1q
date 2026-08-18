---
Status: active
Last-reviewed: 2026-08-15
Authority: sbirs_sensor 算法登记与实现边界
Answers: SBIRS 用了哪些算法、各自实现到什么地步、边界在哪、哪些刻意不实现
---

# SBIRS 算法登记

本文是 SBIRS 算法清单与边界的权威。算法本身的逐步逻辑读代码（`src/sbirs_sensor/`）；本文只回答
"用没用/到哪步/为什么不做"。模块级边界（输出归属、电源、能力决策）见 [boundaries.md](boundaries.md)，
数据流与时序见 [data-flow.md](data-flow.md)。

## 算法登记表

| 算法 | 意图（一句话） | 实现状态 | 证据 |
|---|---|---|---|
| 目标状态机 | 7 状态管理 WFOV 发现→NFOV 捕获→持续跟踪全过程 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_state_machine_test] |
| WFOV 扫描搜索 | 推进扫描相位，执行几何/SNR 门控，输出带误差位置 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test] |
| NFOV 首次捕获 | WFOV cue 生成指向，限速 ATP 稳定后窗口+SNR 判捕获 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test] |
| 逐通道 ATP | 按 NFOV 通道保存光轴，限速推进，判定 settled/timeout | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_pointing_coordinator_test] |
| NFOV 持续跟踪 | 捕获后闭环 ATP + 几何/SNR 门 + 滤波/真值驱动 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test] |
| EKF 滤波跟踪 | 6 维 CV 状态 / 2 维角度量测的扩展卡尔曼滤波 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_ekf_baseline_test] |
| IMM(EKF) 滤波 | 多模型交互，全场景 RMSE 改善 28-55% | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_imm_evaluation_test] |
| Cue 预测 | 角度域两点 CV 提前量补偿 cue 延迟 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_cue_predictor_test] |
| NFOV 资源调度 | 多通道并发锁定，按 SNR/距离/target_id 排序 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_scheduler_test] |
| 地球遮挡门控 | 有限线段射线-地球球体判别穿地视线 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_foundation_test] |
| Foundation 物理链路 | 辐射强度/透过率/接收功率/噪声/SNR 标量链 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_foundation_test] |
| 当前时刻最大探测距离 | WFOV 检测门限反解 d_max(t)=sqrt(I·A·τ_opt·τ_eff·η·t/(N·SNR_th))，逐目标进归属层 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_radiative_transfer_test] |
| 气象衰减 | 查表+加权叠加得透过率衰减因子 | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_environment_model_test] |
| 误差模型 | 5 类误差（轨道/姿态/视场高斯 + 折射/滞后确定性；滞后随相对视线角速度 v_t−v_sat） | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_error_model_test] |
| 时间相关指向扰动 | 整星共模 + 逐通道 GM + 振动的 Gauss-Markov | 生产可用 | [evidence: tests/unit/sbirs_sensor/sbirs_pointing_disturbance_test] |
| 安装指向与稳定链 | 卫星姿态(Body→ECI)∘安装角(Body→Sensor)∘扫描指向合成实际光轴；体/惯性双稳定；传感器系限位 | 生产可用（阶段 2，2026-08-17） | [evidence: tests/unit/sbirs_sensor/sbirs_boresight_chain_test] |
| 安装失准误差模型 | 安装失准（常值偏置 + 运行期一次随机抽取的常值微扰）合成进 boresight 链影响实际光轴与门控；与量测域误差及时变扰动谱正交 | 生产可用（阶段 3，2026-08-17） | [evidence: tests/unit/sbirs_sensor/sbirs_boresight_chain_test]、[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test] |
| 2-D 俯仰栅格扫描 | 俯仰栅格（el 起止+步进角）与方位环扫正交组合；锯齿单向推进、行内相位与既有一致、行末回绕；输出行中心 ECI 俯仰 | 生产可用（阶段 4，2026-08-17） | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test] |
| ECEF→ECI 旋转（GMST） | 周期入口按 UTC 儒略日算 GMST，把卫星/目标位置与卫星/目标速度（含 ω×r 输运项）旋到 ECI | 生产可用（共享域 1q/coordinate/inertial_transform.h，仅 SBIRS 消费） | [evidence: tests/unit/sbirs_sensor/sbirs_eci_transform_test] |
| WFOV 地面覆盖区投影 | 实际扫描中心 ±半视场四角经指向链到 ECI，与地球圆球交会（最近正根），交点旋回 ECEF 取地心经纬度；指向太空的角记 miss | 生产可用（仅验收日志消费，2026-08-18） | [evidence: tests/unit/sbirs_sensor/sbirs_foundation_test] |
| 焦平面脱靶量映射 | 目标相对 NFOV 指向中心的逐轴角差经 x=f·tan(Δaz)、y=f·tan(Δel) 映射为米/像素脱靶量 | 生产可用（仅验收日志消费，2026-08-18） | [evidence: tests/unit/sbirs_sensor/sbirs_foundation_test] |
| 宽窄切换连续命中门 | 逐目标累计连续 WFOV 门通过次数，达到 `wide_to_narrow_required_consecutive_hits`（默认 1）才允许进入 NFOV 调度；门失败/消失清零，捕获清零 | 生产可用（默认 1 行为逐位不变，2026-08-18） | [evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test] |

## 安装指向与稳定链（阶段 2）

- **意图**：对齐 AR 的 `platform_attitude + mount_angles + scan_center` 链路，把卫星姿态与
  传感器安装角引入 SBIRS 内部光轴几何；输出 az/el **保持 ECI 极坐标参考不变**（安装矩阵只
  影响内部光轴，消费方兼容）。
- **实现边界**：
  1. 组合关系：`actual_boresight = attitude(Body→ECI) ∘ mount(Body→Sensor) ∘ scan(传感器系)`；
     旋转矩阵由公共库 `oneq::coordinate::{BuildRotationMatrix, Compose, Inverse}` 合成
     （Z-Y-X 欧拉，正 pitch = 正仰角），传感器系 az/el 与 ECI 采用同一
     `(cos(el)cos(az), cos(el)sin(az), sin(el))` 约定。
  2. 姿态为周期输入（Body→ECI，必填，零欧拉合法 = 体轴对齐 ECI）；安装角为初始化静态配置
     （`SbirsOrientationConfig::mount_angles_deg`），不进 RuntimeConfigPatch。
  3. 稳定方式两种：`kBodyStabilized`（默认）——扫描参数（`scan_start_az/span/el`）为传感器系
     角度，姿态/安装直接旋转光轴足迹；`kInertialStabilized`——扫描参数为 ECI 参考方向，先得
     期望 ECI 单位向量再经链路反解到传感器系（`Rᵀ` 旋转），物理上保持惯性方向稳定。
  4. 传感器系扫描限位（默认 az [-180,180]、el [-90,90] 全开）约束 WFOV 扫描中心与 NFOV 命令；
     WFOV 扫描弧段与中心俯仰须在限位窗口内（配置校验拒绝超窗配置）。
  5. 共模/通道扰动与 NFOV settle 误差在**传感器系**叠加后形成实际指向；identity 链（零姿态 +
     零安装角）下全部数值与历史逐位一致（203 例既有单测回归网）。
  6. 门控作用点迁移：WFOV 门/越界诊断、NFOV 跟踪窗口与首捕窗口、NFOV 命令/初始 LOS 全部改在
     传感器系执行；限位够不到时 actuator 停在限位边缘（AR 同款静默钳制语义），失败走既有
     `kNfovAcquisitionFailed`/跟踪丢门路径。
  7. EKF/cue/5 类量测误差/d_max 保持 ECI 参考不动（量测噪声不随安装链走；安装失准角误差是
     阶段 3 范畴）。
- **反直觉点（参考系分化）**：门控在传感器系、输出在 ECI——非零姿态下目标 ECI az/el 与传感器系
  az/el 不同，但 raw output 仍报 ECI 参考（客户契约）；"探测与否"由传感器系几何决定，"报哪里"
  由 ECI 惯性参考决定。
- **COMMON 收敛决策**：本链复用公共姿态原语（`oneq::coordinate`），未为 SBIRS 引入 Eigen 内核；
  惯性稳定反解仅 3 行矩阵运算，暂不抽 `src/common/` 内核——待第三模块需要同语义时再按
  `src/common/radar/ScanScheduleRuntime.h` 先例收敛（决策登记，避免过早抽象）。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_boresight_chain_test]
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]

## 安装失准误差模型（阶段 3）

- **意图**：在阶段 2 合成链上叠加安装失准角误差，使实际光轴足迹与门控携带静态安装误差；
  与量测域 `attitude_sigma_deg`（时刻输出误差）和指向扰动共模（时变 GM）**谱正交**，
  "避免双重计模"的落点即此三域边界。
- **实现边界**：
  1. 组合关系扩展：`actual_boresight = attitude(Body→ECI) ∘ mount(Body→Sensor)
     ∘ misalignment⁻¹ ∘ scan(传感器系)`——失准作用于传感器系内，等效安装偏置微扰
     （与 mount 同语义不同来源）；旋转矩阵合成复用 `oneq::coordinate`，链保持纯几何
     （`SbirsBoresightChain` 无随机源/时间演化状态，失准总量由 pipeline 抽好传入）。
  2. 配置域：`SbirsOrientationConfig::misalignment`（`SbirsMisalignmentModel`，静态
     会话配置，不进 RuntimeConfigPatch）——常值偏置 `bias_deg`（Z-Y-X，deg）+
     随机微扰 1-σ `random_sigma_deg` + 独立种子 `random_seed`；默认全零 = 既有行为
     逐位不变（226 例单测回归网）。
  3. **时间结构（关键设计）**：随机微扰每次运行抽取一次（pipeline 构造/ApplyConfig 时，
     `DrawMisalignmentTotal` 用配置种子做每轴一次 N(0,σ) 抽取），运行内为常值——
     静态 vs 扰动共模的时变 GM（tau>0）谱不重叠，天然无双重计模；同种子确定性重抽
     保证 replay 可复现与确定性 continuation（运行期失准进 pipeline 快照，
     `Capture/RestoreRuntimeState` 往返）。
  4. 作用域：只影响内部光轴几何（WFOV 门/NFOV ATP/限位钳制/输出扫描方位随链）；**不污染
     量测输出、不进 `BuildMeasurementCovariance`**（量测 RSS 仍只含 orbit/attitude/fov +
     折射 + 滞后）。
  5. 校验：bias 三分量有限、`random_sigma_deg` 非负有限（`kInvalidMisalignment`）；
     `random_seed` 不校验（0 归一化到 1，沿既有种子惯例）。
- **与既有误差域的边界（避免双重计模）**：

| 误差域 | 时间结构 | 作用点 | 流/种子 |
|---|---|---|---|
| 量测域（orbit/attitude/fov/range sigma） | 每周期白噪声重抽 | 量测输出（`ApplyAngularErrorModel`） | `error_model.random_seed` 派生 3 流 |
| 指向扰动共模/通道 | 时变 GM（tau>0）+ 确定性振动 | 传感器系实际指向（加法叠加） | `pointing_disturbance.random_seed` 派生 |
| 安装失准（阶段 3） | 运行内常值（一次抽取） | boresight 链合成（旋转合成） | `orientation.misalignment.random_seed` |

- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_boresight_chain_test]
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]

## 目标状态机（7 状态）

- **意图**：每个目标独立维护一个状态机实例（以 `target_id` 为键），管理 WFOV 发现 → NFOV 首次捕获 →
  三种互斥跟踪模式的全过程。这是 SBIRS 区别于 EOS（一次性 SNR 判定）的核心。
- **状态枚举**：

| 状态 | 含义 | 该状态下输出 |
|---|---|---|
| `Undetected` | 初始或未被任何视场发现 | 不输出 |
| `WideCandidate` | WFOV 已发现，等待 NFOV 调度 | WFOV 检测记录 |
| `AwaitingNfovAcquisition` | 已预留 NFOV 通道，推进 cue 和 ATP | slewing 时输出 WFOV；settled 后视捕获结果 |
| `StrictTruthAssistedTracking` | 真值 LOS 驱动闭环 ATP | 捕获成功起输出真值；失视时 coasting |
| `SensorLikeTruthAssistedTracking` | 真值 LOS 驱动，成功观测用独立误差流 | 门通过时输出带误差角度/诊断距离 |
| `EstimatedTracking` | EKF/IMM 预测 LOS 驱动，门通过后才消费量测 | 门通过时输出滤波估计；失视时预测 coasting |
| `Lost` | 目标从场景消失或传感器关闭 | 不输出 |

- **实现边界**：
  1. 首次 NFOV 捕获**必须**使用 WFOV 输出的带误差位置，不得直接用真值位置。
  2. 捕获成功后由 `SbirsTrackingMode` 三选一（默认 `kEstimated`）；三者使用独立状态枚举与稳定
     attribution source。
  3. 捕获失败不是独立状态；失败转移回 `WideCandidate` 并清除交接上下文。
  4. 三个 tracking 状态→`WideCandidate` 的丢锁条件：几何或 SNR 门连续失败达
     `nfov_tracking_gate_loss_cycles`（默认 2）。
  5. `max_concurrent_nfov_locks > 1` 时多个目标可同时处于捕获/跟踪状态，各占独立 NFOV 通道。
  6. 状态机是跨周期累积状态；snapshot/restore 由 pipeline 自身拥有，mutation 前验证全部 cross-owned
     状态后原子恢复。
- **反直觉点**：两个真值辅助态的命令来源均是真值，但实际 actuator LOS、NFOV 窗口与 SNR 门同样决定
  是否存在有效量测——它**不是绕过光轴动力学和可见性的理想化输出路径**。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_state_machine_test]

### 状态转移条件

| 起点 → 终点 | 触发条件 |
|---|---|
| `Undetected` → `WideCandidate` | WFOV 视场内且 SNR ≥ 门限 |
| `WideCandidate` → `AwaitingNfovAcquisition` | 空闲 NFOV 通道且优先级胜出 |
| `AwaitingNfovAcquisition` 自循环 | ATP 未 settled 且等待 < `180°/max_slew_rate` |
| `AwaitingNfovAcquisition` → Truth/Estimated | ATP settled 后窗口覆盖延迟真值 LOS + SNR 达标 |
| `AwaitingNfovAcquisition` → `WideCandidate` | 窗口/SNR 失败或 ATP timeout |
| tracking 自循环 | 门通过或失败次数未达阈值 |
| tracking → `WideCandidate` | 连续失败达 `nfov_tracking_gate_loss_cycles` |
| 任意 → `Lost` | 目标消失或传感器关闭 |

## WFOV 宽视场多目标搜索

- **意图**：每周期对输入场景所有目标执行 WFOV 扫描判断，输出带误差位置供 NFOV 首次捕获。
- **实现边界**：
  1. 几何门控顺序：地球遮挡 → WFOV FOV 门控 → 范围门控 → SNR 计算。
  2. 扫描相位用 `std::fmod` 常数时间推进；`span=360°` 跨 ±180° 不产生数轴断点。
  3. WFOV 搜索只处理输入场景中显式给出的目标列表，不从图像像素生成新目标。
  4. WFOV 带误差位置是仿真观测/cue，不是目标真值，也不是外部 target identity。
  5. `kWideSearch` 只产生 channel=`-1` 的 WFOV 观测，不分配 NFOV。
  6. 扫描参数参考系（阶段 2 起）：体稳定下 `scan_start_az/scan_center_el` 为**传感器系**角度
     （零姿态 + 零安装角下与 ECI 一致）；惯性稳定下为 ECI 参考方向，经链路反解到传感器系。
     共模扰动与扫描限位在传感器系叠加/钳制后形成实际扫描中心。
  7. 2-D 俯仰栅格（阶段 4 起）：`scan_el_start_deg/scan_el_span_deg/scan_el_step_deg` 与既有
     方位环扫正交组合；`scan_el_span_deg=0` 默认单行模式（行中心恒为 `scan_center_el_deg`，
     既有行为逐位不变）。行数 = `1 + floor(span/step)`，行中心 = `el_start + row·step`；
     锯齿单向推进——每行从方位起点同向扫描，行内相位跨过 `scan_span` 时行索引步进、方位
     相位归零，行末回绕。校验强制 `step ≤ wide_field_fov_el_deg`（无隙覆盖预算）；全幅面
     完成时间 ≈ `row_count × span/scan_rate` 秒。输出 `scan_elevation_rad` 为当前行中心
     合成光轴的 ECI 俯仰（与 `scan_azimuth_rad` 同参考系）。
- **反直觉点（扇区 patch 的相位处理）**：扇区 patch 后，当前绝对方位仍位于新有向半开区间时重算 phase
  并保持指向，否则 phase 归零转到新起点；2-D 栅格 patch 下旧行中心 el 映射到新栅格最近行
  （不在新栅格内则归零行）；rate-only patch 保持 phase。从 SearchAndStare 切入时释放
  NFOV/pointing/filter 绑定但**保留** scan phase、测量随机流和已有 WFOV cue 历史。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]

## NFOV 首次捕获

- **意图**：对进入 `AwaitingNfovAcquisition` 的目标跨周期执行限速指向与首次捕获判定。
- **实现边界**：
  1. 指向生成用 `SbirsCuePredictor`：角度域两点有限差分估计角速度，生成
     `u_cmd = u_measured + angular_rate × narrow_cue_latency_s`。命令**不消费目标真值速度**。
  2. 窗口判定以 actuator 当前 LOS 为中心，叠加静态 `narrow_pointing_settle_error_deg`；
     阶段 2 起窗口判定在**传感器系**执行（命令 = 实际指向经链转换的传感器系 az/el，
     delayed truth = 延迟真值 ECI los 经链转换；identity 链下与历史逐位一致）。
  3. cue 延迟对真实 LOS 的评估按延迟后相对几何线性平移：
     `(p_target + v_target·τ) − (p_satellite + v_satellite·τ)`，卫星位移同样计入
     （卫星速度必填，2026-08-17 起），不做积分轨道传播。
  4. 失败/超时清除交接并回退 `WideCandidate`，产出 `capture_failure_reason` 诊断归属（不进 raw output）。
- **反直觉点（cue 命令和 eligibility truth 都在 latency horizon 上评估）**：目标真实 LOS 在
  `narrow_cue_latency_s > 0` 时按延迟时间线性外推后重算（目标与卫星位移都计入；目标无速度时
  仅卫星位移生效）。因此捕获判定仍受 WFOV 误差、目标运动、卫星运动、
  cue 延迟和 NFOV 视场大小影响，**不会因为窗口中心直接取测量值而恒成立**。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_cue_predictor_test]

### CA cue predictor（刻意拒绝接线）

五样本角度二次最小二乘 CA 已完成 characterization：108 个无噪声持续加速组合中聚合 RMS 从 CV 的
`0.068965 deg` 降为数值零；但在 `dt=0.1s`、`latency=0.5s`、`sigma=0.01deg` 的恒速场景，CA RMS/P95 为
`0.141490/0.279187 deg`，**劣于** CV 的 `0.074132/0.141619 deg`，捕获率也从 `73.91%` 降至 `41.30%`。
因未通过标称噪声零回退门，当前拒绝生产接线，不新增 CV/CA 配置、schema 或自动切换。

[evidence: tests/unit/sbirs_sensor/sbirs_cue_ca_characterization_test]

## 逐通道 ATP（光轴执行）

- **意图**：`SbirsPointingActuator` 把光轴表示为单位 LOS 向量，沿球面最短路径限速推进。
- **实现边界**：
  1. 限速 `narrow_pointing_max_slew_rate_deg_per_sec × dt`；一步可到达时直接落到命令向量，禁止过冲。
  2. `narrow_pointing_settle_tolerance_deg` 判断 settled（默认 0.01 deg）；速率默认 30 deg/s。ATP 始终启用。
  3. settled 后施加静态 `narrow_pointing_settle_error_deg`——这是与速率/容差独立的第三个物理量。
  4. coordinator 以 `channel_id` 持有 actuator、绑定目标、捕获等待时间和跟踪门连续失败计数。
  5. 未 settled 且累计等待达 `180°/max_slew_rate` 时产生 `kNfovPointingTimeout`，释放资源并禁止同周期重调度。
  6. 首次捕获成功后不释放绑定，而是清零捕获等待并晋级为 tracking。
- **反直觉点（释放语义）**：普通释放只解除目标绑定并保留末次 LOS；standby 才清空全部通道运行态。
  每周期先推进已有 awaiting 目标，再调度新候选，因此释放的容量可在同周期供其他目标使用。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pointing_coordinator_test]
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pointing_actuator_test]

## NFOV 持续跟踪

- **意图**：捕获成功后，三种互斥模式共享同一条闭环可见性链：命令 LOS → actuator 限速推进 → 有效 NFOV
  中心 → 几何门 + SNR 门。
- **实现边界**：
  1. 单周期门失败不产生 raw 量测，进入 `Coasting` 诊断并保持通道。
  2. 连续失败达 `nfov_tracking_gate_loss_cycles`（默认 2，必须 ≥1）才正式丢锁。
  3. 三个 tracking 状态→`WideCandidate` 时输出 `kNfovTrackingGateLost` 并释放 scheduler/ATP/滤波状态。
  4. runtime Strict↔Sensor-like 原地转换并保留 NFOV 锁；任一 Truth↔Estimated 释放不兼容跟踪状态。
  5. 阶段 2 起命令链：ECI 命令 az/el（真值或 EKF 预测）→ 传感器系 + 限位钳制 → 链合成 ECI 单位
     向量驱动 actuator（actuator 限速转向在 ECI 单位向量域，参考系无关，快照不变）；跟踪窗口在
     传感器系比较。
- **反直觉点（Estimated 的严格因果顺序）**：predict → actuator advance → geometry/SNR gate → correct。
  门失败时滤波器只预测、不采样量测、不产生 NIS，并清零连续 NIS 超限计数。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]

### 两种 TruthAssisted 模式

1. **StrictTruthAssisted**：成功记录使用当前真值（受物理门约束的 oracle）。
2. **SensorLikeTruthAssisted**：命令和状态转移与 Strict 相同；成功周期用独立随机子流生成带误差角度和
   诊断距离。SNR 仍是物理链原值。
3. **无反馈边界**：Sensor-like 带噪输出不参与指向、几何/SNR 判门或状态转移；门失败/coasting 不消费
   Sensor-like 随机数。
4. **随机流隔离**：同一 measurement seed 经固定混合与 domain tag 派生 WFOV/cue、Estimated 量测和
   Sensor-like 输出三个子流，snapshot 分别保存。一个模式的采样不改变另一链路的未来样本。

## EKF 滤波测量跟踪（EstimatedTracking）

- **意图**：对 `EstimatedTracking` 状态的目标（默认），用扩展卡尔曼滤波做测量跟踪。
- **实现边界**：
  1. 状态空间：6 维 ECI 恒速 `[x,vx,y,vy,z,vz]`（CV 交错布局），复用 common 的转移模型
     （2026-08 正式变更：输出参考系改为 ECI——pipeline 周期入口按 GMST 把真值从 ECEF
     旋转到 ECI，滤波状态随之在 ECI 中演化；CV 对惯性系恒速目标更贴合，不含 ECEF 科氏耦合）。
  2. 量测模型：2 维 `[az,el]`（弧度，ECI 极坐标），被动红外不测距。选纯 2 维角度而非 3 维 + 大 R 屏蔽 range。
  3. 初始化（方案 A）：状态均值用输入场景真值经 ECEF→ECI 旋转后的位置 + 速度（速度含
     ω×r 输运项，见 `include/1q/coordinate/inertial_transform.h`）；初始协方差由
     `initial_position_std_m`/`initial_velocity_std_m_per_s` 构造。
  4. 每周期因果闭环：SetSatellitePosition（ECI）→ predict → actuator advance → geometry/SNR gate → correct（动态 R）。
  5. 输出角度用滤波估计的 ECI 位置 → 相对卫星 LOS → az/el（弧度，az∈[0,2π)、el∈[-π/2,π/2]）；
     SNR/range 仍用真值物理链。
  6. runtime patch 对已存在滤波器只让后续周期读取新 R/Q，**不重置**协方差和状态向量。
- **反直觉点（真值初始化简化）**：EKF 首次捕获用真值位置初始化均值，后续 update 才用带误差测量——
  这是仿真的 track initiation 简化，不得描述为完全无真值辅助的真实载荷跟踪器（见 SBIRS-OQ-4）。
- **反直觉点（2 维量测的选择理由）**：选纯 2 维 `[az,el]` 而非 3 维 `[az,el,range]`+大 R——后者把不可观测
  的 range 塞进量测向量需要用极大 R 屏蔽，语义不干净；纯 2 维直接表达"被动红外只测角"。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_ekf_baseline_test]
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]

### NIS 诊断与丢锁

1. NIS = `innovationᵀ · R⁻¹ · innovation`，χ² 自由度=量测维数（2），95% 门限 ≈5.99。
2. NIS 持续偏高→模型失配（CV 在助推段失配）；持续偏低→R 偏大。
3. 默认 NIS 只读不触发动作。当 `nis_gate_loss_cycles > 0` 时，连续 NIS 超门限达配置周期数后释放锁定。
4. NIS 丢锁是确定性门限计数，不做概率抽样（行业标准做法，AR 模块同理）。

[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]

## 滤波后端选型

当前接线 EKF 和 IMM(EKF) 两个生产后端，由 `estimated_backend` 选择。顶层模式与估计后端是两个正交枚举。

| 后端 | 生产状态 | 当前阻塞 | 重新进入门 |
|------|:---:|------|------|
| EKF | live | — | 当前默认 |
| IMM(EKF) | live | — | 显式配置；全场景 RMSE 改善 28-55% |
| SRIF | evaluation only | 线性 H helper 对 6D/2D 量测不兼容 | 先支持非线性量测，再证明 covariance/LLT 可复现失稳 |
| UDKF | evaluation only | 绑定线性 H；UD 分解不解决非线性量测 | 先提供 6D/2D 非线性接口和优于 EKF 的证据 |
| CKF | evaluation only | 仓库没有实现 | 提供 Jacobian 近似误差超门的场景矩阵和独立实现验证 |

- **反直觉点（不做在线自动切换的理由）**：
  1. **可复现性优先于智能性**：在线自动选型使同一想定因阈值微调走不同后端，结果不可比。
  2. **选型决策依赖外部真知**："目标是否机动"等判据仿真期真值已知，泄露到选型逻辑等同作弊。
  3. **可解释性**：工程评审需能追溯到具体后端与参数。

[evidence: tests/unit/sbirs_sensor/sbirs_imm_evaluation_test]

## NFOV 资源调度

- **意图**：`max_concurrent_nfov_locks` 个并发 NFOV 通道，每个通道独立凝视锁定一个目标（默认 1 退化单目标）。
- **实现边界**：
  1. 新目标被选中时分配**最小可用编号**，ATP slewing 与捕获期间保持预留。
  2. 编号分配确定性：相同输入在 replay 中产生相同的目标→通道映射。
  3. 调度器不读取仿真目标名称，只使用状态、SNR、距离和 `target_id`。
  4. `nfov_channel_id` 仅进 attribution 调试层，不进 raw output。
  5. **切换前置条件（2026-08-18）**：新候选进入调度须先满足连续 WFOV 命中门
     `hits >= wide_to_narrow_required_consecutive_hits`（默认 1 行为不变；见"验收派生量"
     节第 5 条）；已锁定/等待捕获目标不受该门影响。
- **优先级规则**：
  1. 已锁定目标优先级最高（持续占用各自通道）。
  2. 新候选按 WFOV IR SNR 从高到低。
  3. SNR 相同按距离从近到远。
  4. 仍相同按 `target_id` 从小到大。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_scheduler_test]
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]（连续命中门控行为）

## ECEF→ECI 惯性旋转（GMST）

- **意图**：SBIRS 输出参考系为 ECI（2026-08 正式变更）。周期入口由输入 `utc_julian_day`
  （UTC 儒略日，缺失即校验拒绝）计算 GMST，把卫星位置/速度（必填）与每个目标的
  位置/速度旋转到 ECI；下游 LOS/az/el/遮挡/SNR/EKF 全链使用同一 ECI 几何。
- **实现边界**：
  1. GMST 用 IAU 1982 近似（Vallado 式 3-47），UT1 ≈ UTC、无章动/极移；对应方位误差
     < 0.004°，符合仿真精度档（实现与边界见 `include/1q/coordinate/inertial_transform.h`）。
  2. 速度含地球自转输运项 v_ECI = R3(θ)·v_ECEF + ω_e × r_ECI（地面静止目标在 ECI 中
     仍有 ~0.46 km/s 速度，不可忽略）。
  3. 旋转保模长与球体相交语义：遮挡/距离门与帧无关；az 平移 GMST、el 不变（绕 z 旋转）。
  4. 输出角度为 ECI 极坐标弧度：az∈[0, 2π)、el∈[-π/2, π/2]；内部角度量纲保持 deg、
     方位对称约定 (-180, 180]，输出边界统一换算。
  5. 共享坐标域提供 `TryComputeGmstRad`/`TryEcefToEci`/`TryEcefVelocityToEci`，
     当前仅 SBIRS 消费（EOS/AR/ESR 不受影响）。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_eci_transform_test]

## 地球遮挡门控

- **意图**：天基传感器视线穿过地球时目标不可观测；WFOV 搜索前的必要几何门控。
- **实现边界**：
  1. 实现使用有限线段射线-地球球体判定（不用无限射线近似），避免方向符号误用。
  2. 地球遮挡在 WFOV FOV 门控和范围门控**之前**执行，避免对穿地视线做无意义 SNR 计算。
  3. 该门控是帧级（遮挡角只依赖卫星位置）与目标级（夹角依赖目标视线方向）的混合。
  4. 只回答 LOS 是否穿过地球球体，不负责地形、云图、临边散射或三维大气廓线。
  5. 不存在"目标位于大气层以下且距离过远"硬 gate；低空/地面目标的路径损耗只通过标量透过率进入 SNR。
  6. 必须使用一致的坐标输入，不能混用局部 FOV 坐标做地球相交判定。SBIRS 管线在
     周期入口把 ECEF 输入统一旋转到 ECI（GMST），遮挡/距离/视场/SNR 全链使用同一
     ECI 几何；旋转保模长与球体相交语义，门控结果与帧无关。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_foundation_test]

## 验收派生量（[SbirsAccept] 日志专用，2026-08-18）

- **意图**：满足需求映射 3.2.1.3 章节（OPIR 宽视场扫描探测与窄视场跟踪探测）的验收信息
  输出：把管线中间量与少量新增派生量经 `SBIRS_ACCEPTANCE_LOG`（CMake 开关
  `ONEQ_ENABLE_SBIRS_ACCEPTANCE_LOG`，默认 OFF）写入项目日志，人读验收材料。
- **派生量与公式**：
  1. **WFOV 地面覆盖区**：实际扫描中心（传感器系，含共模扰动与限位钳制）± 半视场共 4 角，
     每角经 boresight 链合成 ECI 视线后与地球圆球求交（半无限射线最近正根；
     `TryIntersectRayWithSphere`），交点按周期 GMST 旋回 ECEF 取地心经纬度
     （`ComputeGeocentricLatLonDeg`）；圆球模型与遮挡门控同口径（`kEarthRadiusM`），
     指向太空的角记 `miss`。
  2. **驻留时间**：`dwell_s = wide_field_fov_az_deg / scan_rate_deg_per_sec`（方位向扫描
     穿越视场时间；`scan_rate=0` 退化配置记 0）。
  3. **焦平面脱靶量**：目标传感器系角 − NFOV 实际指向角，逐轴 `x = f·tan(Δaz)`、
     `y = f·tan(Δel)`（米），除以像元间距得像素（`ComputeFocalPlaneOffset`）；逐轴独立
     小角投影，非畸变光学模型。
  4. **目标信号能量**：`E = P_received · t_int`（`ComputeSnr` 出参透出，不改 SNR 数值）。
  5. **宽窄切换连续命中计数**：逐目标累计连续通过 WFOV 四门（遮挡/距离/视场/SNR）的
     周期数；任一门失败、目标消失或待机清零；**捕获成功进入跟踪时清零**（丢锁回宽场后
     需重新积累）。进入 NFOV 调度的前置条件为 `hits >= wide_to_narrow_required_consecutive_hits`
     （`SbirsSchedulerConfig`，默认 1 = 单次命中即调度，与既有行为逐位一致）。计数表进
     `SbirsPipelineSnapshot`（capture/restore 完整）。
- **反直觉点**：
  1. 覆盖区四角在**传感器系**加偏移再经链变换（含姿态/安装/失准），不是在 ECI 角度直接
     加偏移——identity 链下两者才等价。
  2. 连续命中计数在**候选创建点**自增（当周期即计 1），默认阈值 1 下新候选当周期即可调度，
     行为与改造前逐位一致。
  3. 驻留时间是配置派生标量（视场/速率），不依赖目标；与逐周期输出配对供验收核对。
- **边界**：全部派生量仅走日志通道，不进 raw output/attribution/DebugView（boundaries.md
  非目标 10）；开关 OFF 时宏与派生计算一并编译剪除。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_foundation_test]
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pipeline_test]

## Foundation 物理链路

- **意图**：目标辐射强度（W/sr）→ 接收功率、路径透过率、背景/探测器噪声和 SNR 门限的标量顺序计算。
- **实现边界**：
  1. foundation 算法可被单元测试直接覆盖，但不作为 public header、SPI 或 runtime plugin 暴露。
  2. 目标红外签名由调用方以辐射强度 `radiant_intensity_w_per_sr` 直接提供（已折算温度/发射率/投影
     面积），接收功率 `P_sig = I_t · A_ap · τ_opt · τ_atm · η / d²`；模块不再做 Planck 换算
     （无温度输入）。
  3. 第一版只实现标量链路，不实现图像帧、像元级背景图或多色分类器。
  4. 标量 SNR 链不做像元级背景（仍无 PSF/MTF/成像几何）；焦平面脱靶量映射所需的
     `focal_length_m`/`detector_pixel_pitch_m` 已加入 public hardware 与 replay schema，
     但仅被验收日志消费（见"验收派生量"节），不进标量探测链路。
  5. WFOV/NFOV 当前共享同一套波段、孔径、透过率、积分时间和噪声参数；两个通道差异由 FOV、调度/指向
     和各自检测门限表达。
  6. 复制自 EOS 的算法允许按天基场景修正常数和几何输入，但必须保持调用面由 `SbirsPipeline` 统一编排。
  7. 当前时刻最大探测距离（合同指标 4，2026-08-17 起）：信号 ∝ 1/d² 且噪声不含距离项，
     由 WFOV 检测门限闭式反解 `d_max = sqrt(I·A_ap·τ_opt·τ_eff·η·t_int/(N_eff·SNR_th))`；
     τ_eff 与 N_eff 取当前周期快照，故逐目标、逐周期变化。输出进归属/诊断层
     （`SbirsDetectionAttributionRecord::max_detection_range_m`），不进 raw output；
     NFOV 门限版可由消费方按 `d_max·sqrt(wide_min/narrow_min)` 推导；SNR 门失败目标的
     issue 消息附带 d_max 数值（人读）。
- **反直觉点**：只有独立成像模型同时具备 PSF/MTF、焦距与像元几何时，才可冻结其物理效应并增加结果
  测试；不得先加占位字段再用任意归一化系数伪装生效。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_foundation_test]
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_noise_model_test]
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_radiative_transfer_test]

## 气象衰减模型

- **意图**：将场景/大气观测映射为透过率衰减因子 `A_total`，作用于路径透过率。
- **实现边界**：
  1. 气象影响查表得各参数独立衰减比例（海浪/天气/温度/湿度/能见度）。
  2. 加权叠加：独立项 `Σ(w_i·A_i)` + 交互项 `k_j·A_p·A_q`（默认 0 关闭）+ 常数修正。
  3. `A_total ∈ [0,1]`，作用于 `τ_eff = τ·(1-A_total)`，只在透过率维度合成一次。
  4. 第一版使用查表和加权叠加，不接入 MODTRAN/LOWTRAN 或三维天气场。
  5. `base_atmospheric_transmittance` 是可标定标量，不声称实现 Beer-Lambert 廓线。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_environment_model_test]

## 误差模型（WFOV 带误差位置）

- **意图**：对 WFOV 输出的带误差位置生成 5 类误差（轨道/姿态/视场高斯 + 折射/滞后确定性）。
- **实现边界**：
  1. 误差叠加作用于 WFOV 输出层、NFOV cue、Estimated 校正量测和 Sensor-like 成功输出；Strict 输出不叠加。
  2. 轨道/姿态/视场三项始终按 RSS 合成为唯一有效角度 1-σ；三项均为 0 表示不施加随机角误差。
  3. 随机源 `SbirsRandomSource`（xorshift32 + Box-Muller）由固定 seed 经固定 domain 派生为 WFOV、Estimated、
     Sensor-like 三个子流，状态分别随 snapshot 持久化。
  4. 折射与动态滞后为确定性公式，滞后输入为相对视线角速度（v_target−v_satellite 推导，
     目标速度未提供时取 0、卫星速度必填）；卫星运动本身扫过视场也产生滞后
     （2026-08-17 起，此前卫星隐含静止）。
  5. 距离误差只用于内部 cue/诊断链路，不进入 raw output。
- **反直觉点**：误差模型生成的是观测/cue 误差，不改变输入目标真值。随机源必须可注入、可 snapshot
  或可由 replay 固定，避免同一 trace 回放产生不同捕获结果。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_error_model_test]

### 时间相关姿态与指向扰动

- **意图**：`SbirsPointingDisturbance` 建模实际光轴扰动——区别于量测域的 `attitude_sigma_deg`。
- **实现边界**：
  1. 两类随机状态采用一阶 Gauss-Markov 精确离散：共模项同周期移动 WFOV 和所有 NFOV 中心；通道项只
     移动对应 NFOV。
  2. 共模状态每个 pipeline 一份；通道状态按物理 `channel_id` 持有，不按目标持有。
  3. 空闲通道仍随仿真时间推进；普通 release/rebind 和无关字段 patch 不重置。
  4. 全部幅值默认 0；没有可追溯设备参数时不提供仓库级非零"真实 SBIRS"常数。
  5. 当前是**传感器系角度坐标系**的小角度扰动（阶段 2 起：WFOV 共模叠加在传感器系扫描中心、
     NFOV 经链转换后叠加在传感器系指向），不含刚体姿态、角速度控制、反作用轮、饱和或机械耦合。
  6. raw output 和滤波 R 不增加扰动字段；`nfov_pointing_error_deg` 表示合成后的总实际误差。
- **证据**：[evidence: tests/unit/sbirs_sensor/sbirs_pointing_disturbance_test]
