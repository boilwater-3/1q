---
Status: draft
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

1、触发：用户给出用例 16「基于标准卡尔曼滤波的目标状态估计」——对天基红外连续方位/俯仰点迹做线性滤波，输出视线角及其变化率，不估三维位置。
2、当前生产 Estimated 跟踪是 6 维 ECI 位置/速度的扩展卡尔曼滤波（EKF：对非线性量测做局部线性化的卡尔曼滤波），量测是非线性 az/el，闭环指向由估计三维位置反算视线。
- **证据**：[evidence: docs/sbirs_sensor/algorithms.md]
- **证据**：[evidence: src/sbirs_sensor/tracking/SbirsTrackingTypes.h]
- **证据**：[evidence: src/sbirs_sensor/pipeline/SbirsTrackingCoordinator.cpp]

术语：标准卡尔曼滤波 = 状态转移和量测都是线性矩阵的 KF；视线角 = 卫星看目标的方位/俯仰；变化率 = 这两个角对时间的导数。

### 待裁定项（四问）

1、**F1 用例 16 与生产估计器不是同一算法**：冻结「当前 EKF 不满足用例 16」。
   能证明：生产状态是 6 维笛卡尔 CV，量测 h(x) 非线性；用例要线性 KF、状态停在角+角速度。
   能否定：生产路径已经是对 [az, el, 变化率] 的线性 KF。
   最小范围：新增实验后端，不改默认 EKF。

2、**F2 不得替换默认 EKF**：冻结「实验 opt-in，默认仍 kEkf」。
   能证明：NFOV 命令由估计三维位置反算 LOS；IMM/replay/校验只认 kEkf/kImm。
   能否定：闭环已经只吃滤波角度，替换无契约风险。
   最小范围：新后端显式配置才走；默认与回归网不变。

3、**F3 状态必须停在视线角及其变化率**：冻结「标准 KF 不得宣称唯一确定三维位置/速度」。
   能证明：用例 16.4 写明单星角度缺距离；当前 6 维均值靠真值初始化（SBIRS-OQ-4）。
   能否定：仅靠标准 KF + 角度点迹即可唯一还原三维。
   最小范围：实验状态 [az, ω_az, el, ω_el]；指向若接入则直接用滤波角。

4、**F4 第一刀不改 raw output 契约**：冻结「变化率先留在滤波器状态/单测，不进 SbirsDetectionRecord」。
   能证明：公开检测记录只有 az/el/SNR；boundaries 禁止把内部量塞进 raw output。
   能否定：用例 16.2 的变化率必须作为客户主输出才能验收。
   最小范围：不改 `SbirsDetectionRecord` / replay schema；变化率由单测读后验。

5、**F5 不能把现成 6/3 线性 KF 工具直接当角度滤波器**：冻结「误接会得到零 H / 单位阵 F」。
   能证明：`KalmanPredictor` 只在 6 维建 CV；`BuildPositionMeasurementMatrix` 只在 6×3 填位置提取。
   能否定：4×2 实例化已提供角度 CV 的 F 和 H。
   最小范围：SBIRS 内自备 4 维角度 CV 的 F/H；不改 common 的 6/3 专用路径。

6、**F6 CuePredictor 不是用例 16**：冻结「两点差分 cue 不能顶替标准 KF」。
   能证明：CuePredictor 无协方差、无量测更新，只服务 WFOV→NFOV 延迟补偿。
   能否定：它已经输出带平滑的角度航迹并驱动 NFOV 持续跟踪。
   最小范围：保留 cue；实验 KF 另接 Estimated 跟踪。

## §1 证据矩阵

| 待裁定项 | 假设（要证明什么） | 证据来源 | 探针/测试（已执行） | 通过条件 | 否定条件 | 建议判定 |
|---|---|---|---|---|---|---|
| F1 用例 16 与生产估计器不是同一算法 | 生产是 6 维 ECI CV + 非线性 az/el 的 EKF，不是角度域线性 KF | `SbirsTrackingTypes.h`；`algorithms.md` EKF 节；`SbirsEstimatedTrackingBackend` | 1、读 facade：状态 6 维、量测 2 维，「h(x) 非线性，走 EKF」，别名 `EkfPredictor`/`EkfUpdater`。2、读算法登记：EKF 与 IMM(EKF) 为 live；SRIF/UDKF/CKF 为 evaluation only。3、读枚举：只有 `kEkf`/`kImm`，默认 `kEkf`。4、读校验：其它 backend 值报 `kInvalidEstimatedTrackingBackend` | 生产路径不是「角+角速度」线性 KF | 生产已对 [az,el,变化率] 做线性更新 | pass |
| F2 不得替换默认 EKF | 替换会改 NFOV 指向闭环和公开后端契约 | `SbirsTrackingCoordinator.cpp`；`SbirsPipeline.cpp`；replay codec | 1、读 `BuildPredictionResult`：从 `mean(0,2,4)` 取三维位置，减卫星位置得 LOS，再算 az/el。2、读 pipeline NFOV 跟踪：命令取 `PredictTarget` 的输出角（即三维反算）。3、读校验与 FlatBuffer：backend 只允许 EKF/IMM | 默认替换会改指向来源与配置/回放契约 | 指向已直接用滤波角且配置不暴露 backend | narrow |
| F3 状态必须停在视线角及其变化率 | 单星角度 + 标准 KF 不能唯一确定三维；当前 6 维可跑是因为真值播种 | 用例 16.4；`open_questions.md` SBIRS-OQ-4；coordinator 初始化 | 1、读 SBIRS-OQ-4：首次捕获用场景真值位置/速度初始化滤波均值，后续才用带误差角度。2、读 coordinator：correct 后仍用三维均值反算输出角。3、用例边界表：不直接输出三维位置/速度/加速度 | 实验后验不得当作三维位置；命令若接入则用滤波 az/el | 有独立距离量测使三维对标准 KF 可观 | pass |
| F4 第一刀不改 raw output 契约 | 变化率是用例输出，但当前客户主输出没有该字段 | `SbirsOutputTypes.h`；`SbirsOutputDebugView.h`；`boundaries.md` 输出规则 | 1、读 `SbirsDetectionRecord`：字段为 detection_id、az、el、SNR、stage、detected。2、读 DebugView 目标快照：同样无角速度。3、读 boundaries：归属/调试不得混入 raw output | 实验验收用单测读滤波器后验即可 | 用户裁定变化率必须进公开检测记录 | narrow |
| F5 不能误接 6/3 线性 KF 工具 | `KalmanPredictor`/`KalmanUpdater` 的 CV 与 H 只服务 6 维位置提取 | `KalmanPredictor.h`；`IKalmanUpdater.h` | 1、读 `BuildTransitionMatrix`/`BuildProcessNoise`：仅 `kStateDim==6` 填 CV；其它维 F=I、Q=0。2、读 `BuildPositionMeasurementMatrix`：仅 6×3 填 H(0,0)/H(1,2)/H(2,4)；其它维零矩阵。3、SBIRS 现用 `EkfUpdater<6,2>`，不走该 H | 4×2 直接实例化得不到角度 CV | 公共工具已为 4×2 角度布局提供 F/H | pass |
| F6 CuePredictor 不是用例 16 | 现有角度两点外推无滤波、不进 Estimated 航迹 | `SbirsCuePredictor.cpp`；`algorithms.md` Cue 预测 | 1、读 `Update`：第二拍用方位差/dt 当角速度，按 cue 延迟外推命令。2、无协方差、无 R、无 Kalman 增益。3、pipeline 只在 WFOV 候选生成 NFOV 命令时调用，NFOV 持续跟踪走 coordinator | cue 不能替代标准 KF 航迹 | cue 已对 NFOV 跟踪做滤波平滑 | pass |

## §2 判定汇总与待裁定问题

### 建议判定

1、**F1 pass**：生产是 6 维 ECI EKF，用例 16 要角度域线性 KF，缺口成立。
2、**F2 narrow**：实验后端 opt-in；默认 kEkf、IMM、TruthAssisted 不动。
3、**F3 pass**：实验状态为 [az, ω_az, el, ω_el]；禁止把后验当三维位置。
4、**F4 narrow**：第一刀不改 `SbirsDetectionRecord`；变化率由单测断言。
5、**F5 pass**：SBIRS 内写 4 维角度 CV；不改 common 6/3 专用 F/H。
6、**F6 pass**：CuePredictor 保持 cue 用途，不升级成该滤波器。

### 推理（非探针直接值）

1、推理：方位角跨 0/2π 时，线性新息会把环绕当成大机动；实验更新必须用最短弧（wrap）处理方位新息。
2、推理：俯仰近 ±90° 时角度 CV 几何变差；GEO 对地凝视主工作区远离该奇异，第一刀可用特征化测试钉住、不先做球坐标流形滤波。

### 建议 Stage B 范围（用户裁定前不写入 §3）

1、允许：`src/sbirs_sensor/tracking/` 新增 4 维角度 CV 线性预测/更新（Joseph 形式）；coordinator 增加显式 backend 分支；`tests/unit/sbirs_sensor/` 新基线（常速率视线、噪声平滑、方位 wrap）；`docs/sbirs_sensor/algorithms.md` 登记为 evaluation/experimental。
2、禁止：改默认 `kEkf`；改 `SbirsDetectionRecord` / DebugView / replay schema；改 common `KalmanPredictor`/`BuildPositionMeasurementMatrix` 的 6/3 行为；替换 CuePredictor；把滤波后验还原成三维位置去驱动物理门或 SNR。
3、行为：量测为带误差 az/el（弧度）；预测 `x ← F x`，F 为两轴 CV；更新 `z − Hx` 中方位走最短弧；输出角取后验 az/el，变化率取后验 ω。
4、验收：新单测；既有 `unit::sbirs_sensor` 在默认 EKF 下回归。

### 需要用户拍板

1、第一刀变化率是否可以只进单测、不进公开检测记录（F4）。
2、实验后端是加 `SbirsEstimatedTrackingBackend` 新枚举值，还是第一刀只在单测里接线、暂不进 pipeline。
3、NFOV 指向在实验 backend 下是否改为直接用滤波 az/el（不再经三维位置反算）。

## §3 冻结契约（用户讨论结束后填写）

<!-- 一行一项：
1、允许范围：模块/目录、类/函数、测试与文档。
2、明确禁止范围：公开头文件、跨模块类型、schema/回放、测试阈值、兼容层。
3、行为边界：输入、输出、错误回退、生命周期。
4、验收门：构建、聚焦测试、契约测试、特征化测试。
5、非目标。
-->

## 修订记录

1、2026-08-27：初始化 Stage A 矩阵（来源：用户用例 16 + 仓库探针）。

## §4 运行记录（Stage C 后填写）

<!-- 1、实现范围。
2、验证命令与结果。
3、权威回写去向：哪个结论写进了哪个文件。
4、残留风险。
5、后续冻结项。
-->
