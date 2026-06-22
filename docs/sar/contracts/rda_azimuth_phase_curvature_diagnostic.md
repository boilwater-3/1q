# SAR RDA 方位相位曲率诊断工程契约

## 1. 目标

为当前 L1 broadside RDA 增加可解释的方位采样诊断，区分：

- 几何多普勒 Nyquist 裕量。
- 每脉冲方位二阶传播相位曲率。
- RDA 相对精确 GBP 的当前质量适用边界。

本阶段只新增 diagnostics，不改变执行路径或拒绝行为。

## 2. 诊断定义

### 方位采样间距

$$
\Delta x_{az} = \frac{v}{PRF}
$$

输出 `azimuth_sample_spacing_m`。

### 每脉冲二阶方位相位曲率

在参考斜距处使用 broadside 二阶近似：

$$
\Delta^2\phi_{az} =
\frac{4\pi \Delta x_{az}^2}{\lambda R_{ref}}
$$

输出 `azimuth_phase_curvature_rad_per_pulse2`。

### 几何多普勒 Nyquist 裕量

使用当前 aperture 首末脉冲相对中心的最大方位位置

$$
x_{edge} = \frac{N_{pulse}-1}{2}\Delta x_{az}
$$

计算参考点最大几何多普勒：

$$
f_{d,max} =
\frac{2v x_{edge}}{\lambda\sqrt{R_{ref}^2+x_{edge}^2}}
$$

以及：

$$
margin_{nyquist} = \frac{PRF/2}{f_{d,max}}
$$

输出 `max_geometric_doppler_hz` 与 `doppler_nyquist_margin`。单脉冲场景的裕量定义为正无穷。

## 3. 输出与 Replay

- 指标进入内部 `RdaDiagnostics`。
- public Session 的 `sar.rda_peak` message 追加方位采样间距、相位曲率和 Nyquist 裕量。
- 现有摘要级 trace/replay 通过 diagnostics 比较保真，无需修改 schema。

## 4. 验收门

- 基线配置诊断与公式逐项一致。
- PRF/速度等间距参数对产生相同采样间距与相位曲率。
- 载频/斜距等曲率参数对产生相同相位曲率。
- 粗间距退化场景仍显示 Nyquist 裕量大于 `1`，诊断不得误报为混叠。
- RDA 输出复图与现有 diagnostics 字段保持不变。
- 默认与 Conan Eigen 3.3.9 SAR 回归、replay、`sar_ci`、`sar_performance` 和 C++11 兼容门继续通过。

## 5. 冻结边界

- 不增加质量警告或结构化拒绝阈值。
- 不启用 Auto，不根据诊断自动切换算法。
- 不改变 RDA linear RCMC 默认值、尺寸门或 public 配置。
- 不宣称该 broadside 参考公式适用于 L2/L3、斜视、聚束或时变 PRF。

## 6. 后续审批

诊断完成后，扩展孔径、目标布局和参数矩阵，才能决定：

1. 是否增加仅诊断级质量风险警告。
2. 是否需要修正 RDA 离散/参考模型。
3. 是否具备任何 Auto 选择证据。
