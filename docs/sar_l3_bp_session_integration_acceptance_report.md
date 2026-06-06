# SAR L3 BP Public Session 接入审批报告

## 1. 审批结论

L3 航路点轨迹与 BP 已完成 public `SarSession` 受控执行闭环，并通过当前平台审批。

- L3 BP 默认关闭，只能由显式 policy 启用。
- Session 按连续 `pulse_id / PRF` 生成 waypoint 轨迹，使用实际 L3 轨迹生成 raw echo 并执行 BP。
- 跨周期 aperture、结构化拒绝、L3/BP diagnostics 和摘要级 replay：通过。
- Auto、runtime patch、全图复矩阵 replay、时变 PRF、尺寸扩展和二阶补偿：继续后置。

## 2. 执行闭环

- public waypoint 使用相对 Session 起点秒数与 LLA；Session 内相对 scene center 转换为 local Cartesian。
- 首周期填满 aperture，后续周期只按 `dt * PRF` 追加脉冲；第二周期验证从 `0.45 s` 连续生成到 `0.50 s`。
- raw echo、latest-N 实际轨迹与 pulse ring buffer 保持同一 aperture 对齐。
- BP 使用实际 L3 轨迹、共享后向投影核心和 `pulse_major` 遍历，输出 `kL3BpImage` / `has_l3_bp_image` 摘要。

## 3. 结构化拒绝

- L3 BP 与 L1-RDA、L2 运动补偿互斥。
- 未启用 raw echo 或 range compression、waypoint 少于两点、首时刻非零、时间非严格递增时，返回 `invalid_l3_bp_config`。
- waypoint 未覆盖所需固定 PRF 脉冲时刻时，返回 `l3_waypoint_coverage`。
- range 或 azimuth 超过 `128` 时，返回 `l3_bp_size_gate`。

## 4. 诊断与 Replay

- L3 周期输出 `sar.l3_trajectory`，记录本周期生成脉冲数及首末脉冲时刻。
- BP 输出 `sar.bp_peak` 与 `sar.bp_traversal`。
- 两周期 L3 trace replay 保真 waypoint 配置、L3 BP policy、输出摘要和 diagnostics，未发现 divergence。
- focused complex image 继续不进入 replay。

## 5. 验证结果

- 默认与 Conan Eigen 3.3.9 构建全部 `Sar*` 单测：各 `73/73 passed`。
- 默认与 Conan Eigen 3.3.9 SAR replay-fast：各 `10/10 passed`。
- 默认与 Conan Eigen 3.3.9 `ctest -L sar_ci`：各 `4/4 passed`。
- 默认 `ctest -L sar_performance`：`1/1 passed`。
- Conan Eigen 3.3.9 `ctest -L sar_cxx11_compat`：`1/1 passed`。
- `git diff --check`：passed。

全仓无目标构建在既有 `flight_dynamic` 测试处被缺失的 JSBSim `FGFDMExec.h` 阻断；SAR 独立目标与审批门均已构建并通过。

## 6. 冻结边界

- public L3 BP 保持显式 opt-in，不进入 Auto 或 runtime patch。
- BP/GBP `128x128` 上限保持不变；RDA Session `1024x1024` 上限不变。
- waypoint 覆盖范围外禁止外推。
- 不开放全图复矩阵、外部逐脉冲轨迹、时变 PRF 调度、快速 BP、并行/GPU、二阶补偿或自聚焦。

## 7. 下一阶段建议

执行 Phase 2 参考级成像与算法对比闭环完成度审计，逐项核对 RDA、GBP、BP、L2/L3 适用边界、public Session 和 replay 证据，再决定下一扩展方向；Auto 继续后置。
