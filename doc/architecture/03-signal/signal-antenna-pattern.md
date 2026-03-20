# Signal 层天线方向图设计说明

本文说明 Signal 层当前“机载雷达天线方向图”建模的工程语义、输入输出与实现边界，重点解释：

- `AntennaPatternModelType`
- `AntennaPatternConfig`
- `EvaluateAntennaPattern(...)`
- `main_beam_gain_db / effective beamwidth / RadarOrientationConfig` 三者的职责关系

## 1. 设计目标

当前方向图模型不是电磁级阵列仿真，而是面向信号探测链路的**工程近似模型**。设计目标只有三个：

1. 让离轴目标的回波功率不再固定使用主瓣峰值增益
2. 让 `commanded_*_beamwidth_deg` 除测角误差外，也能影响离轴目标的增益和 SNR
3. 用尽量少的参数支持主瓣、旁瓣、后瓣和扫描损失

因此当前实现优先保证：

- 与现有 `SignalPipeline -> SignalDetector -> RadarEquations` 链路兼容
- 参数语义单一，不引入重复真值源
- 后续可逐步替换为更复杂的阵列模型

## 2. 职责划分

当前实现把相关参数拆成四层：

| 层级 | 配置/类型 | 职责 |
|------|-----------|------|
| 峰值能力 | `AntennaConfig::main_beam_gain_db` | 波束中心名义峰值增益，作为探测链路唯一峰值增益真值源 |
| 波束宽度 | `nominal_*_beamwidth_deg` + `commanded_*_beamwidth_deg` | 描述当前波束宽度，先经 `ResolveEffectiveBeamwidth(...)` 解析 |
| 指向/扫描 | `RadarOrientationConfig` | 描述扫描中心、驻留中心、机械/电子扫描限位 |
| 方向图形状 | `AntennaPatternConfig` | 描述主瓣近似类型、旁瓣电平、后瓣电平、扫描损失 |

这里最关键的约束是：

- `AntennaPatternConfig` **不再保存峰值增益**
- 峰值增益只认 `main_beam_gain_db`
- 方向图函数只负责“在某个离轴方向上，相对峰值衰减多少”

这样可以避免以后出现：

- 探测配置里一份峰值增益
- 方向图配置里另一份峰值增益
- 两边不一致但系统静默工作

## 3. 当前输入输出链路

当前单目标探测的方向图相关链路为：

```text
TargetFeature.position_x/y/z
  -> TargetLookResolver
  -> BeamControlResolver
     -> ResolveEffectiveBeamwidth(...)
     -> EvaluateAntennaPattern(...)
     -> one_way_antenna_gain_db
  -> SignalDetector
     -> RadarEquations::ComputeEchoPowerWithGain_dBW(...)
     -> snr_db / detection_prob
  -> MeasurementErrorModel
     -> range_error_std_m / angle_error_std_rad
```

其中：

- `look_az_deg / look_el_deg` 表示目标在雷达局部坐标系中的方向
- `effective beamwidth` 表示当前生效的波束宽度
- `EvaluateAntennaPattern(...)` 输出该方向上的单程增益
- 该增益进入单站雷达方程，发射和接收方向当前采用同一单程增益

## 4. 三种方向图模型的工程语义

### 4.1 `kGaussianMainLobe`

工程语义：

- 用平滑主瓣近似离轴衰减
- 适合作为默认模型
- 对主瓣中心附近的小角度偏离较稳定

当前实现里，它和 `kParabolicMainLobe` 共享同一二次型衰减表达：

```text
attenuation_db = 3 * (normalized_az^2 + normalized_el^2)
```

这意味着当前版本里：

- `Gaussian` 和 `Parabolic` 在数值上等价
- 两者语义上先被区分出来，便于后续替换为真正不同的近似公式

### 4.2 `kParabolicMainLobe`

工程语义：

- 用二次型主瓣描述离轴衰减
- 适合工程估算和快速近似
- 便于和“3 dB 半功率波束宽度”直接联动

当前实现里它本质上是“抛物近似主瓣”保留位。后续如果要把 `Gaussian` 做成真正的指数型衰减，而 `Parabolic` 继续保留二次型，这个枚举就可以直接承接。

### 4.3 `kCosinePower`

工程语义：

- 用余弦幂模型近似阵列离轴衰减
- 比简单二次型更接近某些工程阵面模型
- 在波束边缘处比二次模型更自然一些

当前实现中，指数由“半功率点处衰减 3 dB”反推，因此能保证：

- 当离轴角接近半功率波束半宽时，方向图衰减接近 3 dB

## 5. 主瓣、旁瓣、后瓣与扫描损失

`EvaluateAntennaPattern(...)` 当前按以下顺序处理：

1. 先计算扫描损失 `scan_loss_db`
2. 再判断是否落入后瓣区域
3. 若不在后瓣，则判断是否落入主瓣
4. 主瓣内使用“峰值增益 - 主瓣离轴衰减 - 扫描损失”
5. 主瓣外但不在后瓣时，使用固定旁瓣电平

### 5.1 主瓣判定

主瓣判定基于当前有效波束宽度：

```text
|delta_az| <= az_beamwidth / 2
|delta_el| <= el_beamwidth / 2
```

这意味着 `commanded_*_beamwidth_deg` 一旦启用，不仅测角误差会变化，主瓣覆盖范围也会跟着变化。

### 5.2 旁瓣处理

当前旁瓣采用固定截平：

```text
gain = peak_gain + max_sidelobe_level_db - scan_loss_db
```

这是有意为之。当前版本的目标是先把“离轴主瓣”和“旁瓣地板”区分开，而不是细化多级旁瓣包络。

### 5.3 后瓣处理

当前后瓣采用简化规则：

- 若任一轴绝对离轴角大于 `90 deg`，则视为进入后瓣

后瓣增益为：

```text
gain = peak_gain + backlobe_level_db - scan_loss_db
```

这是一种明显简化。它适合当前工程探测仿真，但不应被理解为严格的三维后瓣电磁模型。

### 5.4 扫描损失

当前扫描损失采用二次型近似：

```text
scan_loss_db =
    scan_loss_coeff_db_per_deg2 *
    ((scan_center_az - boresight_offset_az)^2 +
     (scan_center_el - boresight_offset_el)^2)
```

再用 `max_scan_loss_db` 截顶。

工程含义是：

- 扫描中心越偏离阵面法线，损失越大
- 当前只对扫描中心生效，不单独对驻留点和动态波束机动建更细的损失模型

## 6. 当前实现假设

当前实现建立在以下假设上：

1. `TargetFeature.position_x/y/z` 位于雷达局部坐标系
2. `TargetLookResolver` 可直接由局部位置反解目标 `look az/el`
3. 发射与接收方向图相同，因此单站雷达方程用同一个 `one_way_antenna_gain_db` 参与两次增益叠加
4. `BeamControlResolver` 当前通过 `ComputeMountFrameBeamPointing(...)` 计算波束指向
5. 若目标没有 look angle，或未启用方向图功能，则回退到 `main_beam_gain_db`

这些假设在当前代码规模下是合理的，但如果后续引入以下能力，就需要重新审视：

- 发射/接收分离方向图
- 阵元方向图与阵列因子分离
- 稳定模式对真实惯性/地理参考系的影响
- 真实多旁瓣包络
- 极化和频率相关方向图

## 7. 当前限制

当前方向图模型有几个明确限制：

- `Gaussian` 与 `Parabolic` 当前数值实现相同，只是语义保留位
- 主瓣判定采用矩形窗口，而非真实等功率轮廓
- 旁瓣采用固定地板，没有多级旁瓣包络
- 后瓣采用 `> 90 deg` 的简化判定
- 扫描损失只和 `scan_center_deg` 关联，没有单独建 `dwell_center_deg` 影响
- 探测链路当前仍输出单标量 `angle_error_std_rad`，量测协方差在 LOS 正交平面上仍是各向同性近似

## 8. 推荐使用方式

对当前仓库，推荐按下面顺序使用：

1. 在 `AntennaConfig` 中设置 `main_beam_gain_db`
2. 在 `AntennaConfig.pattern` 中设置方向图形状参数
3. 用 `RadarOrientationConfig` 描述扫描窗口与当前波束指向
4. 用 `ResolveEffectiveBeamwidth(...)` 统一解析生效波束宽度
5. 仅在目标具备局部坐标位置时启用 `enable_directional_pattern`

如果只是做第一版工程仿真，推荐默认值思路是：

- `kGaussianMainLobe`
- 较保守的 `max_sidelobe_level_db`
- 小的 `scan_loss_coeff_db_per_deg2`
- 仅在物理化探测链开启时启用方向图修正

## 9. 后续演进方向

后续若继续完善，建议优先级如下：

1. 让 `Gaussian` 与 `Parabolic` 使用不同公式
2. 让扫描损失同时考虑 `scan_center_deg` 和 `dwell_center_deg`
3. 把主瓣判定从矩形窗口升级为等功率轮廓
4. 拆分发射/接收方向图
5. 将方向图修正进一步接入杂波、干扰和 LPI/ECCM 建模

一句话总结：当前方向图模型是“可解释、低参数、便于接入 SignalDetector 的工程近似层”，不是阵列电磁精细模型；它的目标是先把波束宽度、指向和离轴增益对探测结果的影响真实地接入现有信号链。
