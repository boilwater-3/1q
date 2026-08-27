---
Status: frozen
Date: 2026-08-27
Review-Baseline: `evidence/sbirs-angle-standard-kf` @ `6a2dae95`
Authority: 非规范性记录；结论以 docs/common/contract.md、docs/common/session_contract.md
  及各模块 docs/<module>/design.md 为准；与库实现冲突时以库为准。
---

# sbirs-angle-standard-kf：证据矩阵

<!-- 本文档写作规则：
1、证据一律写成一行：- **证据**：[evidence: 路径]，可加 ::符号名；禁止行号。
2、说明简要，一项一行；多个要点用 1、2、3 序号分点分行，禁止大段描述。
3、引用规则时直接写出规则内容，并用证据形式锁定来源文件；禁止写"见xx规则"。
4、面向非专业开发者，用平实中文；术语首次出现时给一句白话解释。
5、探针/测试必须是已实际执行的；无法直接验证的判断以"推理："开头标注。
-->

## §0 背景与待裁定的问题

1、触发：用户下达甲方用例 16「基于标准卡尔曼滤波的目标状态估计」——对天基红外连续方位/俯仰点迹做线性标准卡尔曼滤波，输出滤波后的方位、俯仰及其变化率；明确不输出三维位置/速度。
2、现状：SBIRS 生产 Estimated 跟踪默认是 6 维 ECI 位置/速度的扩展卡尔曼滤波（EKF；对非线性量测在当前点做一阶近似后再走卡尔曼更新），量测是非线性视线角，输出再从估计三维位置反算方位/俯仰。
- **证据**：[evidence: docs/sbirs_sensor/algorithms.md]
- **证据**：[evidence: src/sbirs_sensor/tracking/SbirsTrackingTypes.h]
- **证据**：[evidence: src/sbirs_sensor/pipeline/SbirsTrackingCoordinator.cpp]
- **证据**：[evidence: include/1q/sbirs_sensor/config/SbirsPolicyConfig.h]::SbirsEstimatedTrackingBackend

术语：标准卡尔曼滤波 = 状态转移和量测都是线性矩阵乘法时的卡尔曼滤波（预测 x̂=Fx，量测 z=Hx）。
术语：扩展卡尔曼滤波（EKF）= 量测或动力学非线性时，在当前估计点求导数（Jacobian）再套卡尔曼公式。
术语：视线角 = 卫星看向目标的方位角和俯仰角；被动红外测得到角，测不到距离。
术语：变化率 = 方位角、俯仰角随时间的快慢（°/s 或 rad/s）。

### 待裁定项（四问）

1、**F1 用例 16 与生产估计器不是同一能力**：冻结「当前默认 EKF 不满足用例 16 的线性角度域标准 KF」。
   能证明：生产状态是 6 维 ECI `[x,vx,y,vy,z,vz]`，量测 `h(x)` 为非线性 az/el；用例要线性滤 `[az, ω_az, el, ω_el]`。
   能否定：生产路径已经是角度+变化率的线性 KF（与代码不符）。
   最小范围：新增实验后端，不改默认 EKF。

2、**F2 不得替换生产默认 EKF**：冻结「实验后端 opt-in；默认仍 `kEkf`」。
   能证明：NFOV 闭环用估计三维位置反算 LOS 驱动指向；默认 `estimated_backend=kEkf`；校验只认 `kEkf`/`kImm`。
   能否定：指向与输出已只依赖角度、替换无契约风险（与代码不符）。
   最小范围：加 `estimated_backend` 枚举值；场景默认不变。

3、**F3 状态必须停在视线角及其变化率**：冻结「标准 KF 后验不是三维位置/速度；单星角度不能用线性 KF 唯一确定三维」。
   能证明：用例 16.4 明文禁止；当前三维均值靠真值初始化（SBIRS-OQ-4）。
   能否定：仅凭 az/el 线性 KF 能唯一还原三维（与可观测性及开放议题不符）。
   最小范围：实验状态 `[az, ω_az, el, ω_el]`；指向命令直接用滤波角，不经三维。

4、**F4 第一刀不改公开 raw output**：冻结「变化率先留在滤波器状态和单测；不给 `SbirsDetectionRecord` 加角速度字段」。
   能证明：raw output 只有 `azimuth_rad`/`elevation_rad`；模块边界禁止把内部量塞进 raw output。
   能否定：消费方合同已要求 raw 带变化率（本次用例未给出公开 DTO 变更单）。
   最小范围：公开检测记录仍只报滤波后 az/el；变化率走测试与内部状态。

5、**F5 不得把现有 6 维笛卡尔线性 KF 工具直接当成角度 KF**：冻结「误接 `KalmanPredictor`/`KalmanUpdater` 的 6/3 专用 F/H 会静默得到错误估计器」。
   能证明：CV 的 F/Q 仅 `kStateDim==6`；位置提取 H 仅 `6×3`；其它维度 F=I、Q=0、H=0。
   能否定：公共线性 KF 已支持 4 维角度 CV 与 2 维角度 H（与代码不符）。
   最小范围：SBIRS 内自备 4 维角度 F/H；不改公共 6 维笛卡尔路径。

6、**F6 CuePredictor 不是用例 16**：冻结「两点角度差分只服务 WFOV→NFOV cue，不是带协方差的标准 KF 航迹」。
   能证明：CuePredictor 无 P/Q/R、无更新公式，只保留上一拍角度。
   能否定：CuePredictor 已输出滤波平滑航迹并进 Estimated 闭环（与代码不符）。
   最小范围：CuePredictor 不动；另写角度标准 KF。

7、**F7 方位角过零是真实风险**：冻结「线性角度 KF 必须对方位新息做最短弧（环绕），否则过 0/2π 会当成大跳变」。
   能证明：raw az 范围 `[0,2π)`；CuePredictor 已对方位差做 `NormalizeAzimuth`；笛卡尔 EKF 无此问题。
   能否定：实验量测改用不环绕的无界角（当前输出契约不是这样）。
   最小范围：实验更新器方位新息走最短弧；俯仰钳制在 `[-π/2, π/2]`。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| F1 用例 16 与生产估计器不是同一能力 | 生产是 6 维 ECI CV + 非线性 az/el 量测的 EKF；用例 16 要线性标准 KF，状态为视线角及其变化率 | `algorithms.md` EKF 登记；`SbirsTrackingTypes.h` 6/2 EKF 别名；`SbirsAngleMeasurementModel` 注释「h(x) 非线性，走 EKF」 | 1、读算法登记表：EKF「6 维 CV 状态 / 2 维角度量测的扩展卡尔曼滤波」，生产可用。2、读 `SbirsEkfPredictor`/`SbirsEkfUpdater` 别名指向 `EkfPredictor`/`EkfUpdater`。3、读 `SbirsTrackingCoordinator`：非 IMM 分支构造 `SbirsEkfPredictor`/`SbirsEkfUpdater`。4、读 `estimated_backend` 仅 `kEkf`/`kImm`，默认 `kEkf`。5、全库 `src/sbirs_sensor` 无 `KalmanUpdater` 实例化 | 生产路径确认是非线性角度量测 EKF，不是角度域线性 KF | 生产已是 `[az,ω_az,el,ω_el]` 线性 KF | pass |
| F2 不得替换生产默认 EKF | 替换会改 NFOV 指向闭环与默认配置契约；实验必须 opt-in | `SbirsPipeline` Estimated 用预测 az/el 驱动 ATP；`BuildPredictionResult` 从 6 维位置反算 LOS；校验只认两枚举 | 1、读 `PredictTarget`：EKF 预测后 `BuildPredictionResult` 取 `mean(0/2/4)` 当三维位置，减卫星位置得 LOS，再算 az/el。2、读 pipeline NFOV 跟踪：`command_azimuth_deg = prediction.output_azimuth_deg`。3、读 `ValidateSbirsSessionConfig`：backend 不是 `kEkf`/`kImm` 即校验错误。4、读 `examples/basic_config/sbirs.json`：`"estimated_backend": "kEkf"`。5、读算法「滤波后端选型」：EKF 当前默认；IMM 显式配置 | 默认与闭环依赖 6 维 EKF；新后端只能新增、不能顶替默认 | 指向已只吃角度状态、默认可整体换成线性 KF（与代码不符） | narrow |
| F3 状态必须停在视线角及其变化率 | 单星被动红外只有角度；线性 KF 不能唯一确定三维；当前三维均值来自真值初始化 | 用例 16.4；`SBIRS-OQ-4`；`SbirsTrackingCoordinator` 输出从三维位置反算 | 1、读 `open_questions.md` SBIRS-OQ-4：首次捕获用场景真值 ECEF 位置/速度初始化滤波均值，后续才用带误差角度。2、读 `algorithms.md`：反直觉点写明不得描述为完全无真值辅助的真实载荷跟踪器。3、读 `CorrectTarget`：后验仍 6 维，输出 az/el 由估计位置相对卫星 LOS 计算。4、用例 16.4：输出视线角及其变化状态，不直接输出三维位置、速度和加速度 | 实验后验只含角与角速率；禁止把标准 KF 后验当作三维位置 | 能证明仅凭连续 az/el 线性 KF 可唯一还原三维（与 OQ-4 及用例 16.4 冲突） | pass |
| F4 第一刀不改公开 raw output | 用例要变化率，但公开检测记录与调试视图都没有角速度字段；改 raw output 是公开契约 | `SbirsDetectionRecord`；`SbirsDebugTargetState`；`boundaries.md` 输出规则 | 1、读 `SbirsDetectionRecord`：字段为 `detection_id`/`azimuth_rad`/`elevation_rad`/`infrared_snr_linear`/`observation_stage`/`detected`，无 rate。2、读 `SbirsDebugTargetState`：同样只有 az/el。3、读 `boundaries.md`：仿真归属与内部状态不得混入 `SbirsOutputFrame`。4、`include/1q/sbirs_sensor` 无 `azimuth_rate`/`elevation_rate` 公开字段 | 实验第一刀不扩公开 DTO；变化率由滤波器状态与单测覆盖 | 已有公开角速度字段或合同要求必须改 raw（本次未给出） | narrow |
| F5 不得误接 6 维笛卡尔线性 KF 工具 | 公共 `KalmanPredictor`/`KalmanUpdater` 只实现 6 维位置 CV 与 3 维位置提取；4 维角度实例化会得到 F=I、Q=0、H=0 | `KalmanPredictor.h`；`IKalmanUpdater.h`；AR/RIR 均实例化 `<6,3>` | 1、读 `BuildTransitionMatrix`：仅 `kStateDim==6` 写 `F(0,1)/F(2,3)/F(4,5)=dt`，否则单位阵。2、读 `BuildProcessNoise`：仅 6 维写 Q，否则零矩阵。3、读 `BuildPositionMeasurementMatrix`：仅 `6×3` 置 `H(0,0)/H(1,2)/H(2,4)=1`，否则零矩阵。4、读 AR/RIR：`KalmanPredictor<6,3>` + `KalmanUpdater<6,3>`。5、SBIRS IMM 指针类型是 `<6,2>` 的 EKF，不是线性 `KalmanUpdater` | 实验必须自备 4 维角度 F 与 2 维角度 H；禁止改公共 6 维笛卡尔行为 | 公共线性 KF 已对 4/2 角度布局提供 CV 与 H（与代码不符） | pass |
| F6 CuePredictor 不是用例 16 | Cue 是无协方差的两点角度外推，只补偿 WFOV→NFOV 延迟，不进 Estimated 滤波 | `SbirsCuePredictor`；`algorithms.md` Cue 预测 | 1、读 `SbirsCuePredictor::Update`：有上一拍则 `ω=(θ_k-θ_{k-1})/dt`，命令 `θ+ω·latency`；只存上一拍 az/el。2、无协方差、无 K、无 Joseph 更新。3、读 pipeline：cue 用于调度候选 `command_azimuth_deg`；NFOV 持续跟踪走 `tracking_coordinator_`。4、算法登记「Cue 预测」与「EKF 滤波跟踪」分列 | CuePredictor 保持 cue 用途；用例 16 另写标准 KF | CuePredictor 已是带 P 的标准 KF 并驱动 Estimated 输出（与代码不符） | pass |
| F7 方位角过零是真实风险 | 公开 az ∈ `[0,2π)`；线性新息 `z-Hx` 在过零处可到近 2π；笛卡尔 EKF 无此问题 | `SbirsDetectionRecord` az 范围；CuePredictor 已归一化方位差 | 1、读检测记录注释：az `[0,2π)`，el `[-π/2,π/2]`。2、读 `SbirsCuePredictor.cpp`：`NormalizeAzimuth` 把方位差折到 `[-180,180]` 再除以 dt。3、读 `EkfUpdater`：新息 `measurement - h(x)`，h 由 LOS 算 az/el，无最短弧（EKF 状态在笛卡尔，过零不表现为状态跳变）。4、推理：4 维角度状态把 az 当实数，不过最短弧则过零一次会把新息看成近 360° 机动 | 实验更新器方位新息必须最短弧；俯仰钳制 | 实验改用不环绕角定义且与现有 `[0,2π)` 输出契约一致（当前不是） | pass |

## §2 判定汇总与待裁定问题

### 建议判定

1、**F1 pass**：生产是 6 维 ECI EKF，不是用例 16 的角度域线性标准 KF。
2、**F2 narrow**：只加实验后端；默认 `kEkf`、既有场景、IMM 路径不变。
3、**F3 pass**：实验状态为 `[az, ω_az, el, ω_el]`；标准 KF 后验不得当作三维位置。
4、**F4 narrow**：第一刀不改 `SbirsDetectionRecord`；变化率留在滤波器状态与单测。
5、**F5 pass**：禁止把公共 6/3 线性 KF 工具直接实例化成角度滤波器；SBIRS 内自备 4 维 F/H。
6、**F6 pass**：CuePredictor 不动。
7、**F7 pass**：方位新息做最短弧。

### 推理（非探针直接值）

1、推理：在角度域线性化之后，量测矩阵 H 提取 `[az, el]`，预测 F 为两轴恒速，这时才是用例 16 所说的「标准卡尔曼滤波」。
2、推理：GEO 卫星看助推段目标时，视线角速度缓慢变化，4 维 CV 在短弧上可滤波平滑；长弧机动仍会 NIS 升高，与现有 NIS 丢锁语义兼容。
3、推理：第一刀不把变化率写入 raw output，仍可用滤波后 az/el 作为 Estimated 检测记录，满足用例「滤波后的方位角、俯仰角」；变化率由单测读后验状态覆盖「及其变化率」。

### 建议 Stage B 范围（用户裁定前不写入 §3）

1、允许：
   1. `src/sbirs_sensor/tracking/` 新增 4 维角度 CV 线性预测/更新（Joseph 协方差）。
   2. `SbirsTrackingCoordinator` 增加实验后端分支。
   3. `SbirsEstimatedTrackingBackend` 增加 opt-in 枚举（建议名 `kAngleCvKf`），校验/JSON/replay 同步识别。
   4. `tests/unit/sbirs_sensor/` 新增角度标准 KF 单测（预测、更新、方位环绕、角速率可观测）。
   5. `docs/sbirs_sensor/algorithms.md` 登记实验后端（非默认）。
2、禁止：
   1. 改默认 `kEkf`、既有场景 JSON、IMM。
   2. 改 `SbirsDetectionRecord` / DebugView / 验收 raw 字段。
   3. 改公共 `KalmanPredictor`/`KalmanUpdater` 的 6 维笛卡尔专用 F/H。
   4. 用真值三维位置初始化实验滤波器均值（避免把 OQ-4 简化搬进新后端）。
   5. 把标准 KF 后验解释为三维位置/速度。
3、行为：
   1. 输入：带时标的 az/el 点迹（弧度）与动态 R。
   2. 状态：`[az, ω_az, el, ω_el]`。
   3. 预测：两轴线性 CV；更新：`z=Hx`，方位新息最短弧，Joseph 形式 P。
   4. 初始化：首拍用当前角度、变化率置 0；第二拍可用角度差给变化率先验（与 Cue 两点同构，但是带 P）。
   5. 选中该后端时：NFOV 命令与 Estimated 输出 az/el 直接取滤波角，不经三维 LOS。
4、验收：
   1. 新单测：常值角速度轨迹，滤波 az/el RMSE 低于量测噪声；ω 收敛；过零新息不爆。
   2. `unit::sbirs_sensor` 默认 EKF 回归仍通过。
   3. 配置非法 backend 仍被校验拒绝；`kAngleCvKf` 合法。

### 用户裁定（2026-08-27）

1、F1–F7 全部采纳。
2、实验后端接入 `SbirsTrackingCoordinator`（配置选中才走，默认 EKF 不变）。
3、禁止用真值三维位置初始化实验滤波器均值。
4、本轮不把变化率写入公开检测记录。

## §3 冻结契约

Proven requirement:
- SBIRS Estimated 跟踪增加实验性角度域线性标准卡尔曼滤波后端；默认仍为 6 维 ECI EKF。
- 状态为视线方位/俯仰及其变化率；后验不得当作三维位置或速度。
- 禁止用场景真值三维位置/速度初始化该后端。

Allowed scope:
- Modules/directories:
  - `src/sbirs_sensor/tracking/`
  - `src/sbirs_sensor/pipeline/SbirsTrackingCoordinator.*`
  - `src/sbirs_sensor/pipeline/SbirsPipeline.*`（仅接线与 snapshot 中实验状态表）
  - `src/sbirs_sensor/session/`（校验、replay 枚举编解码）
  - `include/1q/sbirs_sensor/config/SbirsPolicyConfig.h`（仅新增 opt-in 枚举值）
  - `examples/common/config_loaders/sbirs_sensor/`（识别新枚举；不改场景默认）
  - `tests/unit/sbirs_sensor/`、`tests/replay/sbirs_sensor/`（枚举 roundtrip）
  - `docs/sbirs_sensor/algorithms.md`（登记实验后端，非默认）
- Classes/functions:
  - 4 维角度 CV 线性预测/更新（Joseph 协方差）
  - `SbirsTrackingCoordinator` 实验后端分支
  - `SbirsEstimatedTrackingBackend::kAngleCvKf`
- Tests/docs:
  - 常值角速度轨迹：滤波 az/el RMSE 低于量测噪声；ω 收敛；方位过零新息不爆
  - 默认 `kEkf` 回归仍通过
  - 非法 backend 仍拒绝；`kAngleCvKf` 合法

Explicitly out of scope:
- Public headers: 不改 `SbirsDetectionRecord`、DebugView、验收 raw 字段
- Cross-module types: 不改公共 `KalmanPredictor`/`KalmanUpdater` 的 6 维笛卡尔 F/H
- Schema/trace/replay: 只同步识别新枚举；不新增公开角速度字段
- Test thresholds/skips: 不放宽既有 EKF/IMM 阈值
- Compatibility layers: 不替换默认后端；不改既有场景 JSON；不动 CuePredictor；不动 IMM

Behavior boundary:
- Inputs: 带时标的 az/el（弧度）与动态 R
- Outputs: Estimated 检测记录仍只报滤波后 az/el；变化率留在滤波器状态与单测
- Errors/fallback: LLT 失败则后验=预测；方位新息最短弧；俯仰钳制 `[-π/2, π/2]`
- Lifecycle/debug/trace: 首拍用当前角度、变化率置 0（带 P）；第二拍可用角度差给变化率先验；选中该后端时 NFOV 命令与输出 az/el 直接取滤波角，不经三维 LOS
- Init: 禁止真值三维位置/速度写入实验滤波器均值

Acceptance gates:
- Build: 聚焦 `1q_sbirs_sensor_unit_tests`
- Focused tests: 新角度 KF 单测 + `unit::sbirs_sensor`
- Contract tests: 配置校验与 replay 枚举 roundtrip 覆盖 `kAngleCvKf`
- Characterization tests: 本轮不做场景 RMSE 对比（不改默认场景）

Non-goals:
- 替换生产默认 EKF/IMM
- 单星角度标准 KF 输出三维位置/速度/加速度
- 把变化率写入公开检测记录
- 推广公共线性 KF 到任意维度

## 修订记录

1、修订 1（2026-08-27）：脚本初始化骨架；填入 F1–F7 建议判定。来源：用户下达用例 16，并确认在 `evidence/sbirs-angle-standard-kf` 上继续 Stage A。
2、修订 2（2026-08-27）：用户裁定 F1–F7 采纳、接入 coordinator、禁止真值三维初始化、本轮不写公开变化率；冻结 §3。

## §4 运行记录（Stage C 后填写）

<!-- 1、实现范围。
2、验证命令与结果。
3、权威回写去向：哪个结论写进了哪个文件。
4、残留风险。
5、后续冻结项。
-->
