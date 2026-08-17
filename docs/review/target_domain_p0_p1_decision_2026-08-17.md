---
Status: draft
Date: 2026-08-17
Review-Baseline: `main` @ `086a227e`（docs(review): add phased development plan）
Authority: 目标域交付 P0（证据）+ P1（无迹原语）的 Stage A 证据矩阵与冻结契约决策记录。
  本文是证据优先开发模式（contract.md §证据优先开发模式，
  .claude/skills/evidence-first-freeze-contract）在本交付的落点；阶段定义见
  target_domain_development_plan_2026-08-17.md；存量偏离见 open_questions.md
  TARGET-OQ-1..4。非规范性记录，不得替代 contract.md 与各模块 design 文档；
  若与库实现冲突，以库为准。
---

# P0+P1 启动：Stage A 证据矩阵与冻结契约

## 0. 结论速览

- P1（无迹滤波原语）判定 **pass**：库内接口与注入形态（`ITransitionModel`/`IMeasurementModel`
  仅需 `Function`）完全容纳 sigma point 实现，零 public API、零 CMake 接线、消费端零改动。
- P0 双裁定：OQ-3（SBIRS Estimated 输出语义）与 OQ-4（RIR 识别接入）均判定 **narrow**——
  先以 characterization 测试产出客观证据，裁定建议随证据登记，正式裁定待指标签认一并冻结。
- 可达性矩阵判定 **narrow**：以滤波层直连 characterization 测试承载（简化弹道 + 单星几何 +
  Monte-Carlo 扫描），作为需求方指标签认的物理地板依据；不做高保真弹道。

## 1. 现状证据（接口与契约事实）

| 事实 | 内容 | 证据 |
|---|---|---|
| 预测器接口 | `IKalmanPredictor::Predict(const GaussianStateT&, float dt) const` 纯虚 + `UpdateConfig(KalmanPredictorConfig)` 默认空操作 | src/common/estimation/IKalmanPredictor.h |
| 更新器接口 | 两个 `Update` 重载纯虚（无 R 版约定转发 dynamic_R 版）+ `UpdateConfig` 默认空操作 | src/common/estimation/IKalmanUpdater.h:62-80 |
| 非线性注入形态 | `EkfPredictor(const ITransitionModel*, EkfPredictorConfig = {})` / `EkfUpdater(const IMeasurementModel*, EkfUpdaterConfig = {})`，非拥有裸指针；fail-safe 返回先验/后验=预测 | src/common/estimation/EkfFilter.h:169,235 |
| 自定义 config 约定 | Ekf 先例：不重写基类 `UpdateConfig`，config 经构造注入（基类注释明示） | IKalmanPredictor.h L47-48 注释、IKalmanUpdater.h:75-79 |
| 过程噪声单源 | `KalmanPredictor<N,M>::BuildProcessNoise(dt, q)` 静态共享 | src/common/estimation/KalmanPredictor.h |
| IMM 多态接入 | ImmFilter 经基类指针调 `Predict`/`Update`——实现两接口即可零改动接入 | src/common/estimation/ImmFilter.h |
| 数值防护 | `kCovarianceFloor=1e-6f`；LLT 失败 fail-safe + PROJECT_LOG_ERROR（前置中文双行注释） | src/common/numerics/NumericGuard.h、EkfFilter.h:271-279 |
| 库内无无迹实现 | 全库 grep unscented/sigma point 零命中；`Udkf*` 为 UD 分解稳定化（线性 KF 家族） | 2026-08-17 全库审查 |
| 测试零接线 | `tests/unit/common/*_test.cpp` 平铺 GLOB 自动进 `unit::common` 分区 | tests/cmake/partitions/Unit.cmake:26-28 |
| characterization 先例 | CA cue predictor 拒绝证据（73.91%→41.30%）= 固定 seed GTest + RecordProperty 出数字 + 文档回写 | tests/unit/sbirs_sensor/sbirs_cue_ca_characterization_test.cpp |
| SBIRS 会话构造模板 | 五域 config 全默认可构造；JD 取 GMST≈0 使 ECI≡ECEF 便于解析真值角；tracking_mode 从 policy 切、支持单会话热切换 | tests/integration/sbirs_sensor/sbirs_session_test.cpp |
| 库内无弹道构件 | flight_dynamic 为 JSBSim 飞机；全库无 Kepler/弹道传播器 | 2026-08-17 探索结论 |
| RIR 输出契约 | `RirOutputFrame` 仅按内部 `association_key` 的识别结论，无方位/运动学 | include/1q/remote_identification_radar/session/RirOutputTypes.h |

## 2. Stage A 证据矩阵

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|---|---|---|---|---|---|---|
| F1 无迹原语接口容纳性 | sigma point 预测/更新可在不修改任何既有头的前提下实现 IKalman 双接口 | EkfFilter.h 接口事实（§1） | 头文件编译 + `unit::common` 新测试 | 双接口全部 override、Ekf/Udkf/ImmFilter 零改动 | 需要改基类或既有原语才能编译 | pass |
| F2 线性极限正确性 | 线性模型（LinearCv + LinearPosition）下无迹与标准 KF 数值一致 | sigma point 数学性质（线性函数精确传播均值/协方差） | `estimation_unscented_test` LinearModelsMatchStandardKalman | mean/covariance isApprox（容差 1e-4 量级） | 任一分量超容差 | pass |
| F3 Estimated 输出语义 | SBIRS Estimated 模式 raw output 为跨周期平滑估计而非单周期量测 | boundaries.md 输出规则 4 + pipeline 代码路径 | `sbirs_estimated_semantics_characterization_test`（OQ-3） | Estimated 误差序列的离散度/自相关结构显著区别于 Sensor-like（相对关系断言） | 两模式统计结构不可区分（则"平滑估计"论断不成立，OQ-3 改写） | narrow（证据已落地待签认，实测见 §4.1） |
| F4 角度-only 弱可观测地板 | 单星角度-only 短弧段下距离/发射点误差存在数量级放大的物理地板 | 可观测性理论 + 滤波直连实验 | `sbirs_angle_only_reachability_characterization_test`（可达性矩阵） | 短弧距离 RMSE 显著大于长弧（≥5×）、发射点误差 ≥ 同时刻位置误差 | 各弧长档误差同量级（则弱可观测论断在本几何下不成立，需重审指标口径） | narrow（矩阵已实测，实测修正两项原设门，见 §4.2） |
| F5 RIR 识别接入方式 | 三方案中"调用方键映射"（方案 a）为零库内改动且满足本交付 | fusion 关联键契约（调用方保证跨源一致）+ RIR 输出契约（§1） | 文档级能力矩阵（本文 §5.2） | 方案 a 覆盖交付需求且不新增 public seam | 方案 a 无法覆盖（识别结论无法与航迹对应）则升级 b/c | narrow（建议 a，待 P3 立项复核） |

## 3. P1 冻结契约（Frozen Contract）

```text
Proven requirement
  对齐文档 §3.1 裁定：需求"UKF"= 无迹卡尔曼。P2 估计层航迹滤波需要无迹原语；
  库内现无任何无迹实现（§1）。

Allowed scope
  Modules/directories:
    - src/common/estimation/UnscentedTransform.h   （新，内部辅助：sigma 点/权重/重建）
    - src/common/estimation/UnscentedPredictor.h   （新，final : IKalmanPredictor）
    - src/common/estimation/UnscentedUpdater.h     （新，final : IKalmanUpdater）
    - tests/unit/common/estimation_unscented_test.cpp（新，GLOB 自动收编）
  Classes/functions:
    - UnscentedTransformConfig{alpha=1, beta=2, kappa=0}
    - UnscentedPredictor(const ITransitionModel<N>*, UnscentedPredictorConfig = {})
      —— Predict 签名逐字对齐基类；只消费 ITransitionModel::Function（不要求 Jacobian）
    - UnscentedUpdater(const IMeasurementModel<N,M>*, UnscentedUpdaterConfig = {})
      —— 两个 Update 重载对齐基类；sigma 点直过 h(x)，无 H_ 成员
    - Q 复用 KalmanPredictor<N,M>::BuildProcessNoise；R 默认 BuildDefaultMeasurementNoise
  Tests/docs:
    - tests/unit/common/estimation_unscented_test.cpp：
      LinearModelsMatchStandardKalman（硬门）、dynamic-R 路径、非线性模型有限性 +
      与 EkfUpdater 相对比较（弱断言）、权重和=1、null model fail-safe、
      <6,2> 非标准维度实例化、ImmFilter 多态混合接入

Explicitly out of scope
  Public headers:        include/1q 零变更
  Cross-module types:    不新增 SBIRS/RIR/fusion 侧 facade 或 using（消费端接入属 P2）
  Schema/trace/replay:   零变更
  Test thresholds/skips: 不修改既有测试与阈值
  Compatibility layers:  不重写基类 UpdateConfig（Ekf 先例：config 构造注入）
  既有原语:              Ekf*/Udkf*/Srif*/ImmFilter/Kalman* 语义零变更

Behavior boundary
  Inputs:  高斯先验 + dt（Predict）；预测态 + 量测 + 可选 dynamic_R（Update）
  Outputs: GaussianState / KalmanUpdateResult（innovation + S 填充）
  Errors/fallback: null model → PROJECT_LOG_ERROR（中文双行注释）+ 返回先验/后验=预测；
                   LLT 失败 → 同上 fail-safe；协方差对称化 + kCovarianceFloor 对角下限
  Lifecycle/debug/trace: 不接入 trace/replay（纯数值原语，同既有原语）

Acceptance gates
  Build:     目标 1q_common_unit_tests（Windows vcvars + UseEnv 流程）编译通过
  Focused:   ctest -C Debug -R "unit::common" 全绿（含新测试）
  Contract:  不适用（无 public API 变更）
  Characterization: LinearModelsMatchStandardKalman 为正确性硬门

Non-goals
  平方根/UD 稳定化无迹变体（组合实现留 P2 按需评估）；
  弹道系数增广状态；自适应 sigma 参数。
```

## 4. P0 证据设计

### 4.1 实验 A：Estimated vs Sensor-like 输出语义（TARGET-OQ-3）

载体：`tests/unit/sbirs_sensor/sbirs_estimated_semantics_characterization_test.cpp`
（CA cue characterization 先例：固定 `random_seed`、双会话同场景对照、RecordProperty 出数字、
相对关系弱断言）。

设计：模板取 `sbirs_session_test.cpp` 的 MakeSessionConfig/MakeBaseInput（JD 取
GMST≈0），单目标近匀速 + GEO 卫星，error_model 角 σ 非零；双会话仅
`policy.tracking.tracking_mode` 不同（kEstimated / kSensorLikeTruthAssisted）；每周期从
raw output 取 az/el，对 ECEF≈ECI 解析真值角（atan2/asin）。

证据门（实测通过，2026-08-17，`unit::sbirs_sensor`）：
1. Estimated 输出误差 std 0.317° < Sensor-like 0.379°（平滑降方差）；
2. Estimated 逐周期差分 std 0.0097° ≪ Sensor-like 0.117°（后验惯性，>10× 平滑度差）；
3. Estimated lag-1 自相关 0.942 > 0.8。
**附带发现**：Sensor-like 误差亦非白噪声——其 std（0.379°）远大于配置角 σ（RSS≈0.087°），
逐周期差分远小于自身 std，自相关 0.909：来自共模姿态/轨道偏差（慢变偏置）。含义：
估计层即使默认消费 Sensor-like，其 R 建模也须包含共模偏差项（P2 噪声通道设计输入）。

建议结论（待签认冻结）：Estimated 维持"滤波后验即传感器报告值"的装备语义（boundaries
规则 4 不变）；估计层默认消费 Sensor-like 模式输出（量测语义成立，R 含共模偏差项）；
Estimated 的来源标注增强留给 P2 适配器（随噪声通道设计一并落地）。
[evidence: tests/unit/sbirs_sensor/sbirs_estimated_semantics_characterization_test]

### 4.2 实验 B：角度-only 可达性矩阵（指标签认依据）

载体：`tests/unit/sbirs_sensor/sbirs_angle_only_reachability_characterization_test.cpp`
（`sbirs_imm_evaluation_test.cpp` 滤波直连模板 + 本地 mt19937 固定 seed 噪声）。

设计（已执行）：静止单星 + 常重力解析弹道，量测 = 真值角 + 固定种子高斯噪声；滤波 =
无迹（P1 原语）+ 匹配弹道转移模型（误差源隔离为纯可观测性）；先验位置 10 km / 速度
1.5 km/s（代表搜索域，均值=真值不构成信息泄漏）。扫描弧长 {10,30,60,120} s × 角 σ
{5,50,200} µrad × Monte-Carlo 20。

实测矩阵（2026-08-17，`unit::sbirs_sensor`，确定性可复现）：

| 弧长 \ σ | 5 µrad 径向 RMSE | 50 µrad 径向 RMSE | 200 µrad 径向 RMSE |
|---|---|---|---|
| 10 s | 2.68 km | 6.50 km | 5.25 km |
| 30 s | 8.77 km | 1.72 km | 8.42 km |
| 60 s | 18.17 km | 1.26 km | 2.55 km |
| 120 s | 38.73 km | 1.57 km | 1.97 km |

三条实测结论（已编码为测试门）：
1. **σ=50 µrad 列**弧长单调改善（6.5 km → 1.6 km），地板 ~1.2–1.7 km——单静止单星
   角度-only 的距离可达性地板为公里级，百米级指标在 120 s 弧内不可达。
2. **σ=5 µrad 列随弧长发散**：R≈2.5e-11 rad² 与 1e6 m 位置量级组合触及 float 精度
   边缘，距离信息在中间量中丢失——估计层（P2）若要支持 σ≲10 µrad 需 double 中间量
   或状态缩放（工程注记）。
3. **发射点回推无剧放大**（0.07–1.06× 末周期位置误差）：匹配模型下距离误差与速度
   误差相关，回推部分抵消；"回推放大"仅在模型失配场景出现（推演层产品须携带协方差
   的另一理由）。

[evidence: tests/unit/sbirs_sensor/sbirs_angle_only_reachability_characterization_test]

### 4.3 OQ-4：RIR 识别接入能力矩阵（文档级裁定）

| 方案 | 内容 | 库内改动 | 契约风险 | 交付覆盖 |
|---|---|---|---|---|
| a（建议） | 调用方维护 RIR `association_key` ↔ 融合航迹键映射 | 零 | 无（fusion 关联键本就"调用方保证跨源一致"） | 完整 |
| b | RIR 结论适配为 feature-only `DetectionRecord`（key=0 走特征门） | SensorAdapters + 语义裁定 | 特征相似度门与识别置信度语义错位，关联可靠性存疑 | 待探针（P3） |
| c | RIR 扩公开方位/运动学输出通道 | RIR public API（破坏性） | 最重，需独立冻结迁移契约 | 不必要 |

建议：本交付采用方案 a；b/c 不立项。登记 open_questions TARGET-OQ-4 建议结论，正式裁定
随 P3 立项复核（若推演层识别面改变前提再重开）。

## 5. 指标签认表（实测数字已填，2026-08-17；单静止单星 + 匹配模型无迹滤波）

| 观测条件 | 距离误差（径向 RMSE） | 发射点回推（RMSE） | 备注 |
|---|---|---|---|
| 单星，弧长 10 s，σ=50 µrad | 6.50 km | 0.46 km | 位置误差 6.50 km（径向主导） |
| 单星，弧长 30 s，σ=50 µrad | 1.72 km | 0.62 km | 位置误差 1.72 km |
| 单星，弧长 60 s，σ=50 µrad | 1.26 km | 0.84 km | 位置误差 1.26 km |
| 单星，弧长 120 s，σ=50 µrad | 1.57 km | 1.31 km | 地板 ~1.2–1.7 km |
| 单星，弧长 10–120 s，σ=200 µrad | 5.25 → 1.97 km | 0.45–0.95 km | |
| 单星，弧长 10–120 s，σ=5 µrad | 2.68 → 38.73 km（发散） | 同量级 | float 精度地板（§4.2 结论 2），当前管线不支持 σ≲10 µrad |
| 多源融合（SBIRS+AR 位置 / 双星） | P0 不含（P2 起补） | — | 收敛路径预留 |

签认口径：指标必须以本表观测条件为前提签署；单源短弧段百米级发射点/距离指标在
120 s 弧内**不可达**（地板公里级），不得入契约。传感器角 σ=5 µrad 档当前超出
float 管线精度包络，如需求方坚持该档须先立 P2 数值精度冻结项。

## 6. Stage C 回写

| 项 | 实际结果 |
|---|---|
| 实现范围 | P1：UnscentedTransform/UnscentedPredictor/UnscentedUpdater（结构体静态辅助形态——MSVC v141 无法从 Eigen 依赖默认参数推导维度，契约允许范围内的实现形态调整）+ estimation_unscented_test（9 用例）。P0：2 个 characterization 证据测试 |
| 验证命令与结果 | `cmake --build ... --target 1q_common_unit_tests / 1q_sbirs_sensor_unit_tests`（Windows v141+UseEnv 流程）通过；`ctest -C Debug -R "unit::common"` Passed；`ctest -C Debug -R "unit::sbirs_sensor"` Passed（含新证据测试）；无迹 9/9、证据 2/2 |
| 残留风险 | ①无迹原语 float 精度在 σ≲10 µrad 角度场景不足（§4.2 结论 2，P2 冻结项候选）；②Sensor-like 共模偏差项的 R 建模尚未落（P2 噪声通道设计输入）；③OQ-3/OQ-4 正式裁定待需求方指标签认一并冻结 |
| 后续冻结项 | P2：DetectionRecord 噪声通道（含共模偏差）、FusedTarget 运动学扩展、航迹管理；数值精度（double 中间量/状态缩放）评估 |
| P2 终态（2026-08-17） | 噪声通道落地为记录级 `bearing_noise_sigma_rad` + 配置默认（共模偏差项登记为后续：Sensor-like 偏置建模需 TARGET-OQ-3 签认口径）；FusedTarget 运动学/生命周期扩展、逐航迹无迹滤波、M/N 确认、coasting 均落地（`0b1a1d6a`，`unit::fusion` 6 新用例全绿；`enable_track_filtering` 默认关闭零回退） |
| P3 终态（2026-08-17） | target_inference 算法面落地（`9d402196`，`unit::target_inference` 6 用例全绿）：弹道 RK4（含回推）、地表交点发射/落点、敏度误差预算（J·P·Jᵀ + has_uncertainty 诚实标记）、类型先验×证据融合。RIR 接入 = 方案 a（调用方键映射 → type_evidence，零库内改动，TARGET-OQ-4 建议结论可执行） |
| P4 终态（2026-08-17） | 方向纯净度守护 `target_layer_purity_guard`（contract 分区）+ 示例扩链 TargetInferenceComponent（`3a798c29`；Windows 无 spdlog，示例经 v141 语法级验证 `build/vcvars_cl.bat`，完整构建由非 Windows CI 承载） |
| P5 终态（2026-08-17） | 全量验证与回写：ctest 全分区 Debug 跑通（既有基线失败 integration::airborne_radar 0xc0000409 之外全绿，见 CLAUDE.md 勘误记录）；README/开发计划/本文档回写完成 |
