# SAR 时变 PRF 慢时间采样与重采样工程契约

## 1. 目标

为显式非均匀脉冲时刻建立可编码、可验证的慢时间诊断与均匀轴重采样基础。首批只批准
时间轴诊断和线性重采样，不直接批准时变 PRF RDA/CSA/Omega-K 成像。

## 2. 首批输入边界

- 至少 2 个有限、严格递增的显式脉冲时刻 `t[i]`。
- 每个脉冲对应一个复数慢时间样本。
- 首批输出样本数与输入相同。
- 不支持重复/逆序时刻、外推、缺失脉冲修复或 NUFFT。
- 不接入 Session、public runtime patch、schema、trace 或 replay。

## 3. 名义均匀慢时间轴

对 `N` 个显式时刻：

$$
\Delta t_{nominal} = \frac{t[N-1]-t[0]}{N-1}
$$

$$
t_{nominal}[i] = t[0] + i\Delta t_{nominal}
$$

名义 PRF：

$$
PRF_{nominal} = 1/\Delta t_{nominal}
$$

首尾名义时刻必须与输入首尾严格一致。

## 4. 非均匀度诊断

实际间隔：

$$
\Delta t_i=t[i+1]-t[i]
$$

时间偏差：

$$
e_t[i]=t[i]-t_{nominal}[i]
$$

至少记录：

- sample count and duration
- nominal interval / nominal PRF
- minimum / maximum actual interval
- maximum absolute interval deviation from nominal
- interval-deviation RMS
- maximum absolute time-axis deviation
- time-axis-deviation RMS
- uniform-within-tolerance status

首批默认均匀判定容差为 `max(1e-12 s, abs(nominal_interval)*1e-9)`。

## 5. 线性重采样

对每个名义查询时刻，找到 `t[j] <= tq <= t[j+1]`：

$$
y(t_q)=(1-w)y[j]+wy[j+1],\qquad
w=\frac{t_q-t[j]}{t[j+1]-t[j]}
$$

首尾查询必须精确复用首尾输入样本。查询不得外推。输出不得包含 NaN/Inf。

## 6. 独立参考真值

首批使用复数仿射慢时间信号：

$$
y(t)=(a_r+b_rt)+j(a_i+b_it)
$$

线性重采样必须在所有名义查询点精确恢复该解析真值至浮点容差。复指数信号只用于后续
插值误差趋势，不作为首批“精确恢复”断言。

## 7. 首批验收矩阵

1. 均匀输入严格退化，名义轴与输入轴一致。
2. 非均匀输入记录正确的间隔和时间轴偏差。
3. 名义轴首尾与输入首尾一致且严格递增。
4. 复数仿射信号重采样恢复解析真值。
5. 首尾样本严格保持不变。
6. 重复、逆序、非有限时刻和样本数不匹配被拒绝。
7. 输入不变、重复计算确定，默认与 Eigen 3.3.9 C++11 门通过。

## 8. 尚未批准内容

- NUFFT、sinc 或高阶慢时间插值。
- 缺失脉冲检测与修复。
- 重采样后的 RDA/CSA/Omega-K 生产成像。
- 时变 PRF ambiguity、频谱泄漏或质量阈值。
- public API、Session、schema、trace、replay 与 Auto。

## 9. 实现边界

- 下一阶段只实现显式时间轴诊断和复数向量线性重采样。
- 不修改 RDA 默认路径，不执行二维图像重采样。
- 不宣称时变 PRF 成像已获批。
