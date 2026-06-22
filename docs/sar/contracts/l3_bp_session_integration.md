# SAR L3 航路点与 BP Public Session 受控接入契约

## 1. 目标

将已审批的 L3 航路点轨迹与 BP 受控接入 public `SarSession`，保持默认关闭、显式选择、可 replay、可诊断，不改变现有 L1/L2 默认行为。

## 2. 公开配置

### Mission Waypoint

新增公开 `SarWaypointConfig`：

- `time_from_session_start_s`
- `latitude_deg`
- `longitude_deg`
- `altitude_m`

`SarMissionConfig::l3_waypoints` 保存严格递增时间的航路点列表。外部继续使用 LLA，内部相对 `scene_center_*` 转换为 local Cartesian。

### Policy

新增 `enable_l3_bp_imaging`，默认 `false`。

- `false`：不启用 L3/BP。
- `true`：Session 使用 L3 航路点轨迹生成 raw echo，并显式使用 BP 聚焦。

## 3. 时间与轨迹契约

- L3 waypoint 时间相对 Session 起点，首个成功执行周期的时间基准为 `0 s`。
- 当前 public L3 只支持固定 PRF；脉冲时刻为连续 `pulse_id / PRF`。
- waypoint 时间必须覆盖 Session 需要生成的全部脉冲时刻，禁止隐式外推。
- L3 启用时，逐脉冲轨迹完全来自 mission waypoints；cycle input platform 仍用于输入摘要，但不覆盖 L3 脉冲轨迹。
- 航路点少于两个、时间非递增或脉冲时刻超出覆盖范围时，Session 必须结构化拒绝。

## 4. 执行与互斥契约

- L3 BP 必须同时启用 raw echo generation 和 range compression。
- `enable_l3_bp_imaging` 与 `enable_l1_rda_imaging` 互斥。
- `enable_l3_bp_imaging` 与 `enable_l2_motion_compensation` 互斥。
- public L3 不允许自动回退到 RDA，也不允许 Auto 根据轨迹切换算法。
- public L3 BP 严格限制：
  - `range_sample_count <= 128`
  - `azimuth_pulse_count <= 128`
- 超限返回结构化 `l3_bp_size_gate`，不得静默降级。

## 5. 输出与诊断

公开输出摘要新增：

- `SarProcessingStage::kL3BpImage`
- `has_l3_bp_image`

L3 成功周期必须输出：

- `sar.l3_trajectory`
- `sar.bp_peak`
- `sar.bp_traversal`

不开放 focused complex image 全矩阵。

## 6. Replay 契约

- `l3_waypoints` 与 `enable_l3_bp_imaging` 进入 session config replay。
- `kL3BpImage` 与 `has_l3_bp_image` 进入 cycle output replay。
- L3 waypoint 与 BP policy 不进入 runtime patch，防止 aperture 中途改变轨迹或算法。
- 相同配置与输入 replay 必须保持输出摘要和 diagnostics 一致。

## 7. 验收门

- 默认配置和旧 payload 解码后保持 L3/BP 关闭。
- session config round-trip 完整保真 waypoint 和 BP policy。
- 直线 waypoint L3-BP 与内部 GBP 参考一致。
- 折线 L3-BP 在一阶补偿失效区完成聚焦并输出结构化诊断。
- L1/L2 Session 回归保持不变。
- 默认与 Eigen 3.3.9/C++11、SAR CI 和 `128x128` BP 性能门通过。

## 8. 非目标

- 不支持时变 PRF public 调度。
- 不允许运行期修改 waypoint 或 BP policy。
- 不启用 Auto、二阶补偿、自聚焦、快速 BP、并行或 GPU。
- 不扩大 BP/GBP `128x128` 上限。
- 不开放外部逐脉冲轨迹或 L4/6DOF 输入。
