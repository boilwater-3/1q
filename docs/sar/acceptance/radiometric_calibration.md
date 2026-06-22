# SAR 辐射定标内部闭环审批报告

## 1. 审批结论

阶段 58 辐射定标内部闭环已完成当前平台审批。

- 内部标量模块完成单点定标、多点显式权重融合、RCS 反演和辐射误差评估。
- 单点自校准、幅度平方尺度、距离四次方尺度和一致多点融合闭环通过。
- 无效功率、RCS、斜距、权重和未生效定标结果均被拒绝。
- M1 未归一化 GBP 图像可建立定标因子。
- M4 三个隔离目标使用 M1 定标因子反演时，辐射误差均小于 `0.01 dB`。
- `20 dB SNR + 20 dB SCR` 联合干扰下的辐射误差已记录，不冻结有效性阈值。

## 2. 实现范围

- 新增内部模块 `src/sar/calibration/SarRadiometricCalibration.{h,cpp}`。
- 新增基础标量单元测试与现有参考场景聚焦闭环测试。
- 模块使用图像功率、斜距和 RCS 标量输入，不依赖动态矩阵或 public 类型。
- 未修改 public API、Session、runtime patch 或 replay。

## 3. 公式口径

首批图像响应定标因子：

$$
K_{image} = \frac{\sigma_{cal}}{|I_{cal}|^2 R_{cal}^4}
$$

RCS 反演：

$$
\sigma_{measured} = K_{image}|I|^2R^4
$$

该因子只吸收当前确定性 raw echo、匹配滤波和聚焦链路增益，不声明完整真实系统绝对
功率语义。

## 4. 验证结果

- 默认环境定标基础与聚焦闭环：`5/5 passed`。
- Conan Eigen 3.3.9 定标基础与聚焦闭环：`5/5 passed`。
- 默认 Windows Debug 完整 CTest：`25/25 passed`。
- Conan Eigen 3.3.9 `sar_cxx11_compat`：`1/1 passed`。

## 5. 冻结边界

- 当前仅批准内部图像响应定标闭环。
- 不开放 public 配置、Session diagnostics 或 replay 字段。
- 不引入发射功率、天线增益、系统损耗、自动选点、SNR 门或边缘门。
- 不声明真实标定器、实测数据或完整硬件绝对辐射精度。

## 6. 下一阶段建议

进入辐射定标后续接入决策门，审计是否已有足够证据接入受控 public Session
diagnostics，或是否应先扩展系统因子与有效性语义。
