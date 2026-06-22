# SAR L2 Public Session 接入审批报告

## 1. 审批结论

L2 连续扰动轨迹与一阶运动补偿已完成 public `SarSession` 受控接入，并通过当前平台审批。

- 默认 L1 路径保持不变，L2 policy 默认关闭。
- 启用 L2 时使用实际扰动轨迹生成 raw echo，并在 RDA 前强制执行一阶运动补偿。
- 全零扰动严格退化为 L1 输出摘要，并输出零误差诊断。
- 非零扰动、跨周期 aperture 累积和摘要级 replay：通过。
- 多参考点补偿、L3、二阶补偿、自聚焦和 Auto：继续后置。

## 2. 执行闭环

- Session 同步维护 latest-N 理想轨迹、实际轨迹和 raw pulse history。
- 实际轨迹按固定 seed 与三轴速度扰动生成，跨周期从上一实际脉冲连续外推位置。
- raw echo 使用实际轨迹生成。
- L2 policy 启用时，Session 在 RDA 前使用固定参考点
  `(0, nominal_slant_range_m, 0)` 执行一阶包络与相位补偿。
- L2 未同时启用 raw echo generation 和 RDA，或标准差为负值时，返回
  `invalid_l2_motion_compensation_config`。

## 3. 诊断与 Replay

- 非零 L2 周期输出 `sar.l2_trajectory` 与 `sar.motion_compensation`。
- 零扰动 L2 输出的最大/RMS 位置误差和最大/RMS 斜距误差均为零。
- 跨周期测试验证第二周期只追加增量脉冲，理想/实际轨迹与 raw history 保持同一 aperture 对齐。
- L2 policy、三轴标准差和 seed 经 session config replay 保真，回放结果摘要与诊断一致。

## 4. 验证结果

- 默认构建 SAR 聚焦过滤测试：`42/42 passed`。
- Eigen 3.3.9 构建 SAR 聚焦过滤测试：`42/42 passed`。
- 默认与 Eigen 3.3.9 SAR replay 过滤测试：各 `9/9 passed`。
- `ctest -L sar_ci`：`4/4 passed`。
- `ctest -L sar_performance`：`1/1 passed`。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1 passed`。
- `git diff --check`：passed。

## 5. 冻结边界

- L2 默认继续关闭，不开放 runtime patch。
- public Session 继续固定 RDA + linear RCMC。
- Phase 1 public Session `1024x1024` 上限保持不变。
- 不开放逐脉冲实际/理想轨迹 public 输入。
- 不扩大到多参考点、空间变化补偿、L3、二阶补偿、自聚焦或 Auto。

## 6. 下一阶段决策

下一步必须先选择并冻结独立工程契约：

1. 多参考点/空间变化一阶运动补偿。
2. L3 轨迹与二阶残余误差校正。
3. 面向外部逐脉冲轨迹输入的 public/replay 契约。
