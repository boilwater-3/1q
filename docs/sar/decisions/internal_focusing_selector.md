# SAR 内部聚焦选择器后续接入决策

## 1. 决策结论

阶段 69 不批准将内部聚焦建议器接入 `SarSession` diagnostics，也不开放 public Auto。
建议器保持独立内部参考能力。

下一方向选择 CSA 数学与参考真值工程契约，开始补齐长期方案中的 Phase 3 聚焦算法。

## 2. 接入审计

- 建议器需要显式调用目的：常规成像、独立参考或 L3 非直线成像。
- 当前 public Session 没有独立的调用目的输入，接入会迫使 Session 猜测意图。
- Session 当前已经通过显式 policy 选择 RDA 或 L3 BP；建议器 diagnostics 不会改变路径。
- 为只读建议新增 schema/replay 字段会扩大公共表面，但当前缺少直接用户价值。
- public Auto 仍缺少自动执行、失败回退、跨周期稳定性和结构化警告契约。

## 3. 保留边界

- 内部建议器继续用于决策规则测试与未来 Auto 契约参考。
- 不修改 Session、public policy、output、schema、trace 或 replay。
- 不启用自动执行、算法回退或质量阈值选择。

## 4. 下一方向

CSA 相比 Omega-K 不需要 Stolt 插值，可优先冻结二维频率轴、chirp scaling 相位函数、
参考距离与 RDA/GBP 独立比较口径。实现前必须先完成数学契约和参考真值计划。
