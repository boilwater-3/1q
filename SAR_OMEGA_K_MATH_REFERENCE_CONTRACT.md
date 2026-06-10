# SAR Omega-K 数学与参考真值工程契约

## 1. 目标

为 Omega-K 建立可编码、可验证的波数与 Stolt 映射前置契约。现有设计仅给出高层
四步流程，未提供完整参考相位函数，因此首批只批准波数几何、有效域和 Stolt 查询
位置诊断，不直接批准完整 Omega-K 聚焦。

## 2. 首批输入边界

- L1 匀速直线 broadside 条带模式。
- 均匀 PRF 与均匀距离采样。
- 正载频、采样率、PRF、平台速度和参考斜距。
- 未 shift 二维 FFT bin 顺序。
- 不支持 L2/L3、时变 PRF、斜视、聚束、运动补偿或自聚焦。

## 3. 频率轴约定

距离频率 `f_r` 与方位 Doppler `f_a` 使用
`SAR_CSA_MATH_REFERENCE_CONTRACT.md` 已冻结的未 shift 频率轴。偶数长度 Nyquist
bin 位于正频率位置。

## 4. 双程波数几何

双程距离波数：

$$
K_r(f_r) = \frac{4\pi(f_c + f_r)}{c}
$$

方位波数：

$$
K_x(f_a) = \frac{2\pi f_a}{v}
$$

地距/斜距传播波数：

$$
K_z(K_r,K_x) = \sqrt{K_r^2-K_x^2}
$$

有效域要求：

$$
f_c + f_r > 0,\qquad K_r^2-K_x^2 > 0
$$

无效频点必须被计数，不允许产生 NaN/Inf 后继续处理。

## 5. Stolt 查询映射

首批只冻结从均匀目标 `K_z` 网格反求源距离频率查询位置：

$$
K_{r,source} = \sqrt{K_{z,target}^2 + K_x^2}
$$

$$
f_{r,source} = \frac{c K_{r,source}}{4\pi} - f_c
$$

目标 `K_z` 网格由原始未 shift 距离频率轴映射得到：

$$
K_{z,target}(f_r) = \frac{4\pi(f_c+f_r)}{c}
$$

源查询频率超出原始距离频率轴支持区时必须计数并拒绝完整处理。首批基础实现只输出
查询位置与诊断，不执行复数谱插值。

## 6. 首批诊断

至少记录：

- range / azimuth bin count and spacing
- wavelength
- minimum / maximum valid `K_r`, `K_x`, `K_z`
- invalid dispersion point count
- out-of-support Stolt query count
- minimum / maximum source range-frequency query
- maximum absolute Stolt shift in Hz

## 7. 尚未批准的完整 Omega-K 内容

- 参考函数相位与参考距离使用方式。
- bulk azimuth compression 和 residual phase。
- Stolt 插值核、复数幅相误差与归一化。
- 二维 IFFT 后输出相位参考和幅度标定。
- 斜视、聚束、时变 PRF、运动补偿与自聚焦。

不得依据高层四步流程自行补写公式并宣称完整 Omega-K。

## 8. 首批验收矩阵

1. 未 shift 双频率轴与现有 FFT/RDA/CSA 约定一致。
2. `f_a=0` 时 `K_z=K_r`，Stolt shift 为零。
3. `K_z` 与 Stolt 查询关于正负 `f_a` 对称。
4. `abs(f_a)` 增大时 `K_z` 下降、源查询频率和 shift 增大。
5. 色散无效点和越出支持区查询被分别计数。
6. 输出不包含 NaN/Inf，输入不变且重复计算确定。
7. 默认与 Eigen 3.3.9 C++11 门通过。

## 9. 独立参考真值计划

完整 Omega-K 实现前必须提供经审计的参考函数相位样本、固定二维频谱中间结果或可重复
运行的独立参考脚本。只比较最终图像峰值不足以验证参考相位和 Stolt 插值。

## 10. 实现边界

- 下一阶段只实现波数、Stolt 查询位置和诊断基础。
- 不执行复数谱插值，不新增完整 Omega-K 聚焦入口。
- 不接入内部选择器、public API、Session、schema、trace 或 replay。
- 不修改 RDA 默认路径。
