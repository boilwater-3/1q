# SAR L3 Backprojection 内部闭环审批报告

## 1. 审批结论

L3 专用 BP 内部闭环已完成当前平台审批。

- GBP/BP 共享后向投影核心：通过。
- L1 与 L3 相同输入复图逐样本一致：通过。
- BP 在一阶补偿明确失效的 L3 场景中保持 GBP 真值：通过。
- `128x128` BP 独立性能门：通过。
- public Session、BP public 配置、Auto 和尺寸扩展：继续后置。

## 2. 实现边界

- GBP 使用像素优先遍历。
- BP 使用脉冲优先遍历。
- 两者共用距离压缩、双程斜距、linear 距离插值、传播相位、网格和尺寸门。
- 每个像素内部按相同脉冲顺序累加，因此相同输入复图逐样本一致。
- diagnostics 显式记录 `pixel_major` 或 `pulse_major`。

## 3. L3 失效区证据

在孔径末端横向偏移 `12 m` 的 L3 场景中：

- 一阶补偿 RDA 相对 L3-GBP：NRMS `0.386100`，相关系数 `0.925463`。
- BP 与 L3-GBP：复图逐样本一致，NRMS `0.0`，相关系数 `1.0`。

该证据批准 BP 作为内部 L3 非直线轨迹聚焦路径，不批准自动选择或 public 默认切换。

## 4. 性能证据

当前平台 Debug、`128x128`：

- GBP 像素优先：约 `0.183516 s`。
- BP 脉冲优先：约 `0.177757 s`。

两者均通过当前 `< 10 s` 独立性能门。该结果不批准扩大 `128x128` 上限。

## 5. 验证结果

- 默认与 Eigen 3.3.9 L3/GBP/BP/补偿/RDA/Session 过滤测试：各 `35/35 passed`。
- `ctest -L sar_ci`：`4/4 passed`。
- `ctest -L sar_performance`：`1/1 passed`。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1 passed`。
- `git diff --check`：passed。

## 6. 冻结边界

- BP 继续为内部显式路径。
- 不接入 public Session、runtime patch 或 Auto。
- 不实现快速 BP、并行、GPU、时变 PRF 调度、二阶补偿或自聚焦。
- `128x128` BP/GBP 尺寸门保持不变。

## 7. 下一阶段建议

先冻结 L3 航路点与 BP 的受控 public Session 接入契约，明确 waypoint 配置、时间基准、replay、算法选择和尺寸拒绝行为。完成该契约前，不增加 public 字段。
