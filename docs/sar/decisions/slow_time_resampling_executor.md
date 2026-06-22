# SAR 内部慢时间重采样执行器后续接入决策

日期：2026-06-11

## 1. 决策

内部慢时间重采样执行器暂不接入生产 RDA、Session、public API、schema、trace 或
replay。下一阶段建立扩展时变 PRF 重采样质量矩阵契约，补齐多目标、高 Doppler 与
确定性随机抖动证据。

## 2. 审计依据

- `SarSession` 仍按固定硬件 PRF 和连续 pulse id 构造 aperture。
- RDA 配置只接收单一 `prf_hz`，没有显式慢时间轴或重采样执行状态。
- public/schema/replay 没有 expected interval、请求 id、结构化拒绝原因或重采样诊断。
- 当前质量矩阵只覆盖固定单点场景和确定性周期抖动，不足以冻结通用生产阈值。
- 缺失脉冲已批准为明确拒绝，不能由生产链静默插值或修复。

## 3. 接入门结论

- 保持现有固定 PRF Session 和 RDA 默认行为不变。
- 保持内部执行器为显式、无状态、原子输出入口。
- 不增加缺失脉冲修复、稀疏恢复、NUFFT 或自动回退。
- 在扩展质量证据、public 请求语义和 replay 传播契约分别批准前，不进入生产链。

## 4. 下一方向

下一阶段冻结扩展时变 PRF 重采样质量矩阵：

- 使用单目标与多目标参考场景。
- 覆盖中心和高 Doppler 布局。
- 使用固定 seed 的确定性随机小抖动，并保持缺失脉冲为拒绝 case。
- 记录 raw-history NRMS、RDA 图像 NRMS、相干相关和执行器诊断。
- 不修改生产算法、Session、public API 或 replay。
