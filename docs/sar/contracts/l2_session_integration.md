# SAR L2 与一阶运动补偿 Public Session 受控接入契约

## 1. 目标

将已审批的 L2 连续扰动轨迹与一阶运动补偿受控接入 public `SarSession`，保持默认关闭、可 replay、可诊断，不改变现有 L1 默认行为。

## 2. 公开配置

### Policy

- `enable_l2_motion_compensation`，默认 `false`。
- `false`：Session 严格使用现有 L1 轨迹和 RDA 路径。
- `true`：Session 使用 L2 实际轨迹生成 raw echo，并在 RDA 前强制执行一阶运动补偿。

### Mission

- `l2_velocity_error_stddev_x_mps`
- `l2_velocity_error_stddev_y_mps`
- `l2_velocity_error_stddev_z_mps`
- `l2_random_seed`

全部标准差默认 `0.0`。补偿参考点固定为 local Cartesian `(0, nominal_slant_range_m, 0)`。

## 3. 执行契约

- L2 必须同时启用 raw echo generation 和 RDA。
- public Session 不允许 L2 未补偿 raw echo 进入 RDA。
- 全零扰动允许启用，并必须与 L1 输出摘要一致，同时输出零误差补偿诊断。
- 非零扰动时，Session 必须输出：
  - `sar.l2_trajectory`
  - `sar.motion_compensation`
- L2 配置必须进入 session config replay；本阶段不增加运行期 patch，防止跨 aperture 改变轨迹统计特性。

## 4. 验收门

- 默认配置及现有 replay payload 解码后保持 L2 关闭。
- session config replay 完整保真 L2 policy、标准差和 seed。
- 默认 L1 Session 回归通过。
- L2 Session 确实生成非零轨迹误差并执行补偿。
- 相同配置和输入可 replay，摘要一致。
- C++11 + Eigen 3.3.9 和现有尺寸门继续通过。

## 5. 非目标

- 不允许运行期修改 L2 扰动参数。
- 不开放实际/理想逐脉冲轨迹 public 输入。
- 不启用 L3、二阶补偿、自聚焦、Auto 或多参考点补偿。
- 不改变 public Session `1024x1024` 上限。
