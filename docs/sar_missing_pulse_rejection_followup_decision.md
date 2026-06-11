# SAR 缺失脉冲拒绝矩阵后续决策

## 1. 决策结论

缺失脉冲门禁与拒绝矩阵保持内部参考能力，暂不接入生产 RDA 或 Session。下一阶段选择
内部慢时间重采样请求与执行边界契约，将显式时刻、expected interval、门禁和重采样
组合为可审计请求，不扩大 public 表面。

## 2. 生产接入缺口

- Session 没有已审批的显式时变 PRF/重采样请求。
- public/schema/replay 没有 expected interval、拒绝原因和重采样诊断字段。
- RDA 默认路径仍假设固定 PRF，接入会改变现有审批行为。
- 缺失修复、稀疏恢复和 NUFFT 没有独立真值。

## 3. 接入决策

- 不接入 RDA 默认路径、Session、public API、schema、trace 或 replay。
- 不实现缺失修复、补点、稀疏恢复或 NUFFT。
- 允许建立内部显式请求执行器，组合现有门禁和二维重采样，并返回结构化拒绝原因。

## 4. 下一方向

下一阶段冻结内部请求的显式输入、执行状态、拒绝原因、输出原子性和确定性边界。首批
执行器只产出重采样 raw-history，不调用 RDA。
