# SAR RDA 方位相位曲率诊断验收报告

## 1. 审批范围

本阶段为当前 L1 broadside RDA 增加解释性方位采样 diagnostics：

- `azimuth_sample_spacing_m`
- `azimuth_phase_curvature_rad_per_pulse2`
- `max_geometric_doppler_hz`
- `doppler_nyquist_margin`

诊断不参与执行路径、拒绝、质量警告或算法选择。

## 2. 实现结论

- RDA 使用配置参数和实际 aperture 脉冲数计算采样诊断。
- 单脉冲诊断的最大几何 Doppler 定义为 `0`，Nyquist 裕量定义为正无穷。
- RDA 成像入口明确拒绝单脉冲 aperture，避免进入不受支持的 FFT 成像链。
- Session `sar.rda_peak` 记录全部四项指标。
- 现有摘要级 trace/replay 对 diagnostics 文本进行严格比较，无需修改 schema。

## 3. 验收证据

- 基线配置的采样间距、相位曲率、最大几何 Doppler 和 Nyquist 裕量与公式逐项一致。
- `v/PRF` 相同的速度/PRF 参数对产生相同采样间距和相位曲率。
- 等曲率载频/参考斜距参数对由生产 RDA diagnostics 得到相同相位曲率。
- `0.175/0.2 m/pulse` 质量退化场景的生产 Nyquist 裕量仍大于 `1`，没有误报为混叠。
- Session replay 对新增 `sar.rda_peak` diagnostics 无 divergence。

## 4. 回归结果

- 默认构建全部 `Sar*`：92/92 passed。
- Conan Eigen 3.3.9 构建全部 `Sar*`：92/92 passed。
- 默认与 Conan Eigen 3.3.9 `sar_ci`：各 4/4 passed。
- 默认 `sar_performance`：1/1 passed。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：1/1 passed。
- `git diff --check`：passed。

## 5. 审批结论

阶段 42 完成当前平台审批。新增指标只获批作为 broadside L1 RDA 解释性诊断。

当前证据仍不足以批准：

- 质量风险警告阈值。
- 结构化拒绝。
- RDA/GBP/BP Auto 选择。
- 向 L2/L3、斜视、聚束或时变 PRF 外推诊断含义。

下一决策门应扩展孔径长度、目标方位布局和参数组合矩阵，验证相位曲率与质量退化关系是否稳定。
