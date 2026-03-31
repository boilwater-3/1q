# 机载雷达代码审查报告（第二轮）

> 审查范围：`src/airborne_radar/` 全模块代码（以上次 review.md 修复后为基线）

---

## R1 [HIGH] KalmanUpdater / EkfFilter 缺少 LLT 分解状态检查

**文件**：`KalmanUpdater.cpp:38-39`，`EkfFilter.cpp:54-55`

```cpp
const KalmanGainMatrix K =
    result.innovation_covariance.llt().solve(H_ * predicted.covariance).transpose();
```

`ImmFilter::GaussianLogLikelihood`（`ImmFilter.cpp:202`）同样调用 `.llt()` 但**有** `llt.info() != Eigen::Success` 守卫并回退到 `-inf`。而 `KalmanUpdater` 和 `EkfFilter` 直接使用 `.llt().solve()` 结果，未检查分解是否成功。

**影响**：当新息协方差接近奇异时（积累数值误差、退化量测），LLT 分解失败，Kalman 增益退化为垃圾值，导致滤波器发散且无任何日志告警。

---

## R2 [HIGH] FeatureRepository::QueryBestMatch 除零

**文件**：`environment/database/FeatureRepository.cpp:105-108`

```cpp
const float score_sum = std::accumulate(scores.begin(), scores.end(), 0.0f);
result.probability = best_score / score_sum;  // score_sum == 0 时除零
```

当所有记录的距离极大时，`exp(-distance) ≈ 0`，`score_sum` 为 0，产生 NaN/Inf 并返回 `true`。调用方无法区分成功与失败。

---

## R3 [HIGH] AccumulateMultiSourceEccmFacts confidence_weight 下限抵消可信度阈值

**文件**：`SurvivabilityEvaluatorHelpers.cpp:186-191`

```cpp
if (source.confidence < kMinimumCredibleConfidence - 1e-5f) {  // 阈值 0.35
  return;
}
selection->has_credible_multisource_evidence = true;
const float confidence_weight = std::max(source.confidence, 0.5f);  // 下限 0.5
```

通过 0.35 阈值的低置信源（0.35–0.49 区间）被强制提升到 0.5 权重，与高置信源权重差距被人为压缩。这会系统性抬高低置信源对 ECCM 评分的贡献，可能触发不必要的反干扰动作。

---

## R4 [MEDIUM] DistanceMetric::SetInnovationCovariance 无 LLT 分解检查

**文件**：`association/DistanceMetric.cpp:32-33`

```cpp
void FullMahalanobisDistanceMetric::SetInnovationCovariance(const Eigen::Matrix3f& S) {
  llt_.compute(S);  // 未检查 llt_.info()
}
```

与 R1 同类问题。奇异协方差矩阵导致后续 `Compute()` 返回错误的 Mahalanobis 距离，影响关联配对。

---

## R5 [MEDIUM] BoostTrackPool 不防护同指针双重 Release

**文件**：`tracking/BoostTrackPool.cpp:45-64`

当 `in_use_count_ ≥ 2` 时对同一 `TrackState*` 连续调用两次 `Release()`：
- 两次均成功将指针 push 到 `free_list_`
- 后续两次 `Acquire()` 返回**同一指针**
- 两个调用方写入同一 `TrackState` → 数据竞争 / 静默腐败

`in_use_count_ == 0` 的守卫仅防止计数器下溢，不防止同指针重复释放。

---

## R6 [MEDIUM] ScanScheduleResolver `std::isfinite() == 0` 反模式

**文件**：`runtime/ScanScheduleResolver.h:36, 39`

```cpp
if (std::isfinite(center.az_deg) == 0) {
```

`std::isfinite()` 返回非零整数（非严格 1），与 `== 0` 比较虽然当前可行但脆弱。应使用 `!std::isfinite(...)`。

---

## R7 [MEDIUM] SurvivabilityEvaluatorHelpers 负权重泄漏到评分

**文件**：`SurvivabilityEvaluatorHelpers.cpp:192-193, 205-207`

```cpp
const float power_weight = std::min(source.jammer_power_db / kHighJammerPowerDb, 2.0f);
const float js_weight = std::min(source.jammer_to_signal_db / kHighJammerToSignalDb, 2.0f);
...
if (source.jammer_power_db >= kHighJammerPowerDb || ...)
  selection->burnthrough_gain_score += std::max(power_weight, js_weight) * confidence_weight;
```

当 `jammer_power_db` 或 `jammer_to_signal_db` 为负时，`power_weight`/`js_weight` 也为负。条件 L205-206 使用精确比较（无 epsilon），但更重要的是 `std::max(负值, 负值) * 正值` 会产生负的 `burnthrough_gain_score` 增量，反向抵消合法威胁的评分。

---

## R8 [MEDIUM] 评估器顺序 assert 在 Release 构建中静默失效

**文件**：`SurvivabilityEvaluator.cpp:18-19`

```cpp
assert(evaluation_state.threat_assessment_phase_done && ...);
```

`-DNDEBUG` 下 assert 被移除，评估器顺序约束在生产构建中无效。若执行顺序意外改变，`eccm_source_info` 字段将为未初始化/过期状态且无任何告警。

**建议**：改用运行时检查 + `PROJECT_LOG_ERROR` + early return。

---

## R9 [MEDIUM] 双层 hold 周期计数器可能导致不可预测的保持时长

**文件**：
- `SurvivabilityEvaluator.cpp:23-25,36`（`state_store.eccm_hold_cycles_remaining`）
- `EmissionControlEvaluator.cpp:38-40,50`（`state_store.lpi_hold_cycles_remaining`）
- `ControlReducer.cpp:329-330,364,374`（`lpi_hold_cycles_remaining_`/`eccm_hold_cycles_remaining_`）

评估器层和 Reducer 层各维护独立的 hold 计数器，互不同步。两层 hysteresis 叠加后的实际保持时长难以从任一层的配置值直接推断。

---

## R10 [LOW] SurvivabilityEvaluatorHelpers L205-206 缺少 epsilon

**文件**：`SurvivabilityEvaluatorHelpers.cpp:205-206`

```cpp
if (source.jammer_power_db >= kHighJammerPowerDb || ...)  // 无 epsilon
```

对比 L199-202 使用了 `- 1e-5f` epsilon，L205-206 使用精确比较，风格不一致。

---

## R11 [LOW] ExecuteCycle 无效参数时静默返回无日志

**文件**：`pipeline/CycleExecutor.cpp:139-141`

```cpp
if (cycle_context == nullptr || !HasValidRuntime(runtime)) {
  return;  // 无日志
}
```

---

## R12 [LOW] ScanScheduleResolver.h 仍为 header-only

**文件**：`runtime/ScanScheduleResolver.h`（~260 行内联实现）

与已修复的 P12（SignalComponentFactory）同类问题。每个包含此头文件的编译单元重复实例化所有函数。

---

## R13 [LOW] IRadarContext::UpdateRadarControlProfile 空默认实现

**文件**：`include/1q/airborne_radar/core/context/IRadarContext.h:56-58`

```cpp
virtual void UpdateRadarControlProfile(const common::control::RadarControlProfile& profile) {
  (void)profile;
}
```

非纯虚函数使得子类可以"忘记"实现而不产生编译错误。应考虑改为纯虚。

---

## 汇总

| 编号 | 严重度 | 类型 | 文件 |
|------|--------|------|------|
| R1 | HIGH | 数值稳定性 | KalmanUpdater.cpp, EkfFilter.cpp |
| R2 | HIGH | 除零 | FeatureRepository.cpp |
| R3 | HIGH | 逻辑缺陷 | SurvivabilityEvaluatorHelpers.cpp |
| R4 | MEDIUM | 数值稳定性 | DistanceMetric.cpp |
| R5 | MEDIUM | 内存安全 | BoostTrackPool.cpp |
| R6 | MEDIUM | 类型安全 | ScanScheduleResolver.h |
| R7 | MEDIUM | 数值正确性 | SurvivabilityEvaluatorHelpers.cpp |
| R8 | MEDIUM | 合约违规 | SurvivabilityEvaluator.cpp |
| R9 | MEDIUM | 设计 | Evaluators + ControlReducer |
| R10 | LOW | 一致性 | SurvivabilityEvaluatorHelpers.cpp |
| R11 | LOW | 可观测性 | CycleExecutor.cpp |
| R12 | LOW | 编译开销 | ScanScheduleResolver.h |
| R13 | LOW | API 设计 | IRadarContext.h |
