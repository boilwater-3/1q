# SAR L3 航路点轨迹几何审批报告

## 1. 审批结论

L3 航路点轨迹内部几何能力已完成当前平台审批。

- 显式时间航路点与分段线性插值：通过。
- 固定和非均匀显式脉冲时刻：通过。
- 固定 PRF 直线航路点严格退化为 L1：通过。
- 转角命中、航段速度切换和非法时间契约拒绝：通过。
- public Session、L3 成像、二阶补偿、自聚焦和 Auto：继续后置。

## 2. 已批准能力

- 航路点时间严格递增，位置使用 local Cartesian。
- 脉冲时刻严格递增并位于航路点时间覆盖范围内。
- 每个脉冲位置在当前航段执行线性插值，速度等于当前航段常速度。
- 输出保留显式脉冲时刻并生成连续 `pulse_id`。
- 相同输入重复生成逐点一致。

## 3. 验收证据

- `SarGeometryTest.StraightWaypointTrackExactlyMatchesL1Track`
- `SarGeometryTest.WaypointTrackPreservesTurnAndNonuniformPulseTimes`
- `SarGeometryTest.WaypointTrackRejectsInvalidTimeContracts`
- 默认与 Eigen 3.3.9 SAR 聚焦过滤测试：各 `42/42 passed`。
- `ctest -L sar_ci`：`4/4 passed`。
- `ctest -L sar_performance`：`1/1 passed`。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1 passed`。
- `git diff --check`：passed。

## 4. 冻结边界

- 折线转角只作为确定性几何与算法压力基准，不代表物理瞬时转向。
- 当前只消费显式脉冲时刻，不实现时变 PRF 调度器。
- 不接入 public Session、session config 或 replay。
- 不实现 L3 RDA、BP、二阶补偿、自聚焦或 Auto。

## 5. 下一阶段

阶段 22 使用 L3 轨迹生成内部 raw echo，建立理想 L1、L3 经 L1-RDA 和 L3 经 GBP 的同场景退化基线。该阶段只量化问题，不批准 public L3 或二阶补偿。
