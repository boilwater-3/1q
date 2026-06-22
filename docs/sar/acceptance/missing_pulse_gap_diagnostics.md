# SAR 缺失脉冲与最大慢时间间隙诊断验收报告

## 1. 验收范围

本阶段只验收独立慢时间间隙诊断与门禁二维重采样包装入口，不批准缺失修复、RDA 或
Session 接入。

## 2. 实现结果

- 新增 `DiagnoseSlowTimeGaps`，显式接收 expected interval。
- 记录最小/最大间隙、最大间隙比、拒绝间隙数、疑似缺失数和首个拒绝索引。
- `gap ratio >= 1.5` 时保守拒绝。
- 新增门禁二维重采样入口，拒绝后清空输出且不跨间隙插值。
- 原有无门禁基础函数行为保持不变。

## 3. 测试证据

- 默认与 Eigen 3.3.9 `SarMissingPulseGapDiagnosticsTest.*`：各 `5/5` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

测试覆盖均匀、小抖动、恰好 `1.5x` 边界、单/多缺失、累计计数、门禁拒绝和非法输入。

## 4. 审批结论

缺失脉冲与最大慢时间间隙诊断完成当前平台审批。expected interval 来源、生产调用链、
缺失修复和 RDA/Session 接入继续后置。
