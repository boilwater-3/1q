# SAR Phase 2 参考级成像与算法对比工程契约

## 1. 审批目标

Phase 2 建立可复现、可量化的参考级成像与算法对比闭环，为后续聚焦算法扩展提供统一证据。Phase 2 不改变 Phase 1 public Session 默认行为。

## 2. 已批准范围

### Phase 2A

- 确定性点目标参考场景。
- 统一图像质量指标：峰值位置、距离向/方位向 3dB 宽度、PSLR、ISLR、图像熵。
- 只消除全局常数相位的复图像比较。
- 显式 `none/linear/sinc` 内部 RCMC 策略。
- linear/sinc 相同输入的质量与性能对照。

### Phase 2B

- 严格小场景尺寸门下的 GBP 参考算法。
- RDA/GBP 相同场景对比。
- 相位重参考与残余误差诊断。

GBP 内部坐标与输入冻结：

- 使用现有 local Cartesian：图像行对应方位 `x`，列对应地距 `y`，固定成像平面 `z`。
- 输入为与 RDA 相同的 raw pulse history、匹配滤波和逐脉冲平台位置。
- 逐脉冲距离压缩后，按像素双程时延执行线性距离插值，并补偿 `+4*pi*R/lambda` 传播相位。
- GBP 仅作为内部参考算法；不增加 public Session 算法选择接口。

## 3. 冻结边界

- Auto algorithm selector 继续后置，直到 RDA 与 GBP 分别完成独立质量和性能审批。
- Phase 1 public Session 继续固定 linear RCMC。
- Phase 1 public Session `1024x1024` 当前平台上限不变。
- GBP 初始上限为 `128x128`；提高上限必须单独审批。
- 不开放完整聚焦复矩阵 public API，不增加全图复数矩阵 replay。
- L2/L3、运动补偿、自聚焦、辐射定标、HDF5/GeoTIFF 不进入本阶段。

## 4. 数值与比较契约

- RCMC `none` 不改变输入矩阵。
- linear RCMC 保持 Phase 1 行为。
- sinc RCMC 使用有限核插值，核半宽必须位于批准范围内；边界裁剪必须进入 diagnostics。
- 质量改善必须在相同原始回波、相同匹配滤波和相同 RDA 参数下比较。
- 全局相位重参考只能通过单个常数复旋转对齐图像。
- 跨算法形状 NRMS 在两幅图分别单位能量归一化后计算，允许消除算法固有的全局幅度增益差异。
- 若存在空间变化相位误差、相位斜坡或散焦，比较指标必须保留残余误差。

## 5. 验收门

1. 人工构造图像的质量指标具有确定结果。
2. Sinc 对已知带限分数采样输入的误差低于 linear。
3. linear RCMC、public Session 和 Phase 1 SAR CI 无回归。
4. C++11 + Eigen 3.3.9 SAR engine 编译门通过。
5. Sinc 性能代价被独立记录；未形成场景质量改善证据前，不批准成为默认路径。
6. GBP 必须在进入实现前冻结坐标、相位、尺寸和性能契约。

## 6. 明确禁止

- 在第二算法未通过审批前启用 Auto。
- 用全局相位重参考掩盖空间变化误差。
- 因 Sinc 单测通过而直接替换 public Session 的 linear 默认值。
- 通过放宽质量阈值或跳过真实失败获得绿灯。
