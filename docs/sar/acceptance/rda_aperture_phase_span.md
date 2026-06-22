# SAR RDA 孔径二次相位跨度诊断验收报告

## 1. 审批范围

本阶段为当前 L1 broadside RDA 增加：

- `azimuth_quadratic_phase_span_rad`

该指标只作为中心参考点 aperture 二次相位跨度的解释性 diagnostics。

## 2. 实现结论

- 指标由生产 RDA 使用每脉冲相位曲率和实际 aperture 脉冲数计算。
- 单脉冲诊断跨度为 `0`。
- Session `sar.rda_peak` 记录该指标。
- 现有摘要级 replay 对 diagnostics 严格比较，无需修改 schema。

## 3. 验收证据

- 基线值与 `curvature*((N-1)/2)^2` 公式一致。
- 阶段 43 的三组等跨度组合由生产 diagnostics 得到相同跨度。
- 等跨度中心目标 NRMS 差值保持在 `0.03` 内。
- Session message 和 replay 无 divergence。

## 4. 回归结果

- 默认与 Conan Eigen 3.3.9 聚焦诊断测试：各 14/14 passed。
- 默认与 Conan Eigen 3.3.9 全部 `Sar*`：各 93/93 passed。
- 默认与 Conan Eigen 3.3.9 `sar_ci`：各 4/4 passed。
- 默认 `sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

## 5. 审批结论

阶段 45 完成当前平台审批。

当前仍不批准：

- 质量风险警告阈值。
- 结构化拒绝或算法切换。
- Auto。
- 向 L2/L3、斜视、聚束或时变 PRF 外推。

下一决策门应研究目标方位偏置产生的独立误差是否可以由明确几何量解释。
