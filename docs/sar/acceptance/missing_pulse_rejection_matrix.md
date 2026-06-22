# SAR 缺失脉冲拒绝参考矩阵验收报告

## 1. 验收范围

本阶段只验收内部参考链中的缺失脉冲门禁停止语义，不批准缺失修复、生产 RDA 或
Session/public 接入。

## 2. 矩阵结果

- baseline：门禁允许，重采样与 RDA 完成。
- small jitter：最大间隙低于门限，重采样与 RDA 完成。
- 恰好 `1.5x` 边界：门禁拒绝，输出为空。
- 单缺失：估计缺失 1，门禁拒绝，RDA 未尝试。
- 两个相邻缺失：估计缺失 2，门禁拒绝，RDA 未尝试。
- 两个分离缺失：记录两个拒绝间隙并累计缺失 2，RDA 未尝试。

## 3. 验证结果

- 默认 `SarMissingPulseRejectionMatrixTest.*`：`3/3` passed。
- 默认完整 CTest：`25/25` passed。
- Eigen 3.3.9 `sar_unit`：`1/1` CTest entry passed。
- Eigen 3.3.9 `sar_cxx11_compat`：`1/1` passed。
- `git diff --check`：passed。

## 4. 审批结论

缺失脉冲拒绝参考矩阵完成当前平台审批。门禁已证明能在插值和聚焦前停止缺失 case；
缺失修复、生产 RDA 和 Session/public 接入继续后置。
