# SAR 时变 PRF 重采样质量参考矩阵契约

## 1. 目标

使用固定参考点场景量化确定性慢时间抖动经线性重采样后的 raw-history 与 RDA 图像误差，
建立通过区、趋势区和失效区证据。首批不接入生产 RDA。

## 2. 确定性抖动

基于均匀时刻 `t_i=i/PRF`，首尾固定，内部时刻加入：

$$
\delta t_i=A\Delta t\sin(2\pi i/(N-1))
$$

其中 `A` 为相对名义间隔的抖动比例。扰动后时刻必须有限、严格递增。

## 3. 首批参数矩阵

- `A = 0.0`：严格退化基线。
- `A = 0.05`：小抖动候选通过区。
- `A = 0.15`：中等抖动趋势区。
- `A = 0.35`：大抖动失效/边界区。

若参考场景尺寸导致时刻逆序，应明确拒绝该 case，而不是排序修复。

## 4. 参考链

1. 构建均匀参考场景、脉冲位置和 raw-history。
2. 依据抖动时刻在同一 L1 匀速轨迹上重建实际脉冲位置并生成 jittered raw-history。
3. 对 jittered raw-history 执行二维慢时间线性重采样。
4. 比较 resampled raw-history 与均匀 raw-history。
5. 使用相同 RDA 配置分别聚焦均匀与 resampled raw-history，并比较图像。

## 5. 指标

- raw-history 单位能量归一化 NRMS。
- RDA 图像全局相位对齐后的 NRMS。
- RDA 图像相干相关系数。
- 慢时间最大轴偏差和间隔偏差。
- 重采样诊断有效性与均匀状态。

## 6. 首批验收

1. `A=0` raw-history 与图像严格退化。
2. 抖动增大时最大时间偏差严格增加。
3. raw-history NRMS 与图像 NRMS 整体不改善，相关系数整体不提高。
4. 小抖动 case 必须优于大抖动 case。
5. 所有结果确定，非法/逆序时间轴明确拒绝。
6. 默认与 Eigen 3.3.9 C++11 门通过。

首批矩阵先记录测量值，不在实现前猜测通用数值阈值。

## 7. 后置内容

- 缺失脉冲、大间隙、随机抖动和时变 PRF 调度。
- sinc/高阶重采样与 NUFFT。
- RDA、Session、public API、schema、trace、replay 和 Auto 接入。
