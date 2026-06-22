# SAR RDA 孔径二次相位跨度诊断工程契约

## 1. 目标

为当前 L1 broadside RDA 增加 aperture 级二次相位跨度解释性诊断，补足每脉冲相位曲率未包含孔径长度的问题。

本阶段只新增 diagnostics，不增加质量警告、拒绝、算法修复或 Auto。

## 2. 诊断定义

沿用每脉冲二阶方位相位曲率：

$$
\Delta^2\phi_{az} =
\frac{4\pi \Delta x_{az}^2}{\lambda R_{ref}}
$$

使用 aperture 首末脉冲相对中心的半宽脉冲数：

$$
n_{edge} = \frac{N_{pulse}-1}{2}
$$

定义孔径二次相位跨度：

$$
\phi_{span} = \Delta^2\phi_{az} n_{edge}^2
$$

输出 `azimuth_quadratic_phase_span_rad`。

单脉冲场景的跨度定义为 `0`。

## 3. 输出与 Replay

- 指标进入内部 `RdaDiagnostics`。
- public Session 的 `sar.rda_peak` message 追加该指标。
- 现有摘要级 trace/replay 通过 diagnostics 严格比较保真，无需修改 schema。

## 4. 验收门

- 基线配置诊断与公式一致。
- 单脉冲诊断跨度为 `0`。
- `5@0.2` 与 `9@0.1`、`9@0.2` 与 `17@0.1`、`17@0.2` 与 `33@0.1` 产生相同跨度。
- Session message 和 replay 保真。
- 默认与 Conan Eigen 3.3.9 SAR 回归、`sar_ci`、`sar_performance` 和 C++11 兼容门通过。

## 5. 冻结边界

- 该指标描述中心参考点 broadside 二次近似，不覆盖目标方位偏置产生的独立误差。
- 不增加任何经验警告阈值。
- 不增加结构化拒绝、算法切换或 Auto。
- 不外推到 L2/L3、斜视、聚束或时变 PRF。
