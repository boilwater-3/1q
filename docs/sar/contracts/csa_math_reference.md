# SAR CSA 数学与参考真值工程契约

## 1. 目标

为 Chirp Scaling Algorithm 建立可编码、可验证的数学前置契约。现有设计文档只给出
高层处理步骤，未提供完整相位函数，因此首批只批准 CSA 频率几何基础与有效域诊断，
不直接宣称完整 CSA 聚焦已获批。

## 2. 首批输入边界

- L1 匀速直线 broadside 条带模式。
- 均匀 PRF 与均匀距离采样。
- 正载频、采样率、PRF、平台速度和参考斜距。
- raw pulse history 维度完整且非空。
- 不支持 L2/L3、时变 PRF、斜视、聚束、运动补偿或自聚焦。

## 3. 频率轴约定

距离频率轴：

$$
f_r[k] =
\begin{cases}
k f_s/N_r, & k \le N_r/2 \\
(k-N_r) f_s/N_r, & k > N_r/2
\end{cases}
$$

方位 Doppler 频率轴：

$$
f_a[l] =
\begin{cases}
l PRF/N_a, & l \le N_a/2 \\
(l-N_a) PRF/N_a, & l > N_a/2
\end{cases}
$$

轴顺序必须与现有未 shift FFT 输出一致。forward FFT 不归一化，inverse FFT 除以轴长度。

## 4. CSA 频率几何基础

波长：

$$
\lambda = c/f_c
$$

距离徙动几何因子：

$$
D(f_a) = \sqrt{1-\left(\frac{\lambda f_a}{2v}\right)^2}
$$

chirp-scaling 几何系数：

$$
\alpha(f_a) = \frac{1}{D(f_a)} - 1
$$

有效域要求：

$$
\left|\frac{\lambda f_a}{2v}\right| < 1
$$

无效频点必须被诊断，不允许产生 NaN/Inf 后继续处理。

## 5. 首批诊断

至少记录：

- range / azimuth frequency bin count
- range / azimuth frequency spacing
- wavelength
- minimum `D(f_a)`
- maximum absolute `alpha(f_a)`
- invalid Doppler bin count
- valid Doppler limit `2v/lambda` in Hz
- maximum absolute azimuth-axis Doppler in Hz
- Doppler validity margin in Hz: `2v/lambda - max(abs(f_a))`

## 6. 尚未批准的完整 CSA 相位函数

以下内容必须在独立参考真值建立后单独冻结：

- 第一 chirp-scaling 相位函数。
- 距离压缩、SRC 与 RCMC 合并相位函数。
- 方位压缩与残余相位校正。
- 参考距离变化与距离相关 Doppler rate。
- 输出相位参考和幅度归一化。

不得依据高层六步流程自行补写公式并宣称完整 CSA。

## 7. 首批验收矩阵

1. 奇偶长度频率轴与现有 FFT bin 顺序一致。
2. `f_a=0` 时 `D=1`、`alpha=0`。
3. `D` 关于正负 Doppler 对称，`alpha` 同样对称。
4. 接近有效域边界时 `D` 下降、`alpha` 增大。
5. 超出有效域的频点被计数并拒绝完整处理。
6. M1/P 参数矩阵记录 CSA 几何诊断，与现有 RDA Doppler 诊断交叉核对。
7. 默认与 Eigen 3.3.9 C++11 门通过。

## 8. 独立参考真值计划

完整 CSA 实现前必须提供至少一种独立参考：

- 经审计的离线参考脚本与固定复数中间结果；或
- 与公开公式逐项对应的固定相位函数样本；或
- 与 GBP 独立真值对比且包含中间域诊断的确定性场景。

只比较最终图像峰值不足以验证完整 CSA 相位函数。

## 9. 实现边界

- 下一阶段只实现频率轴、几何因子和诊断基础。
- 不新增完整 CSA 聚焦入口、public 算法枚举路径、Session、schema 或 replay。
- 不修改 RDA 默认路径或内部选择器。
- 不启用 Auto、时变 PRF、尺寸扩展或 GPU。
