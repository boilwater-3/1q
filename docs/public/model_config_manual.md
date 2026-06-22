# 对外公开模型配置说明手册

本文面向使用 `include/1q/.../config` 公开头文件的外部调用方，说明 AR、EOS、ESR 三个模块中模型初始化配置与运行期配置补丁的字段语义，以及这些字段会影响模块内部的哪些功能链路。

当前公开配置的主干是四个初始化域：

- `hardware`：装备或传感器固有能力，通常影响物理链路、接收/发射能力、视场或扫描分辨率。
- `mission`：任务态与工作模式，通常影响扫描/指向/工作模式等运行控制。
- `policy`：算法策略，通常影响探测门限、关联、跟踪、生命周期、杂散光等策略选择。
- `environment`：场景环境语义输入，通常影响传播、杂波、干扰、辐射传输或环境服务模型。

运行期补丁类 `*RuntimeConfigPatch` 只描述可在不重建 Session 的情况下更新的白名单字段；初始化固定能力字段通常不在运行期补丁里出现。

## 1. AR 机载雷达配置

### 1.1 初始化聚合：`session::RadarSessionConfig`

来源：`include/1q/airborne_radar/config/RadarSessionConfig.h`

| 字段 | 影响功能 |
| --- | --- |
| `hardware` | 探测链路硬件与物理探测配置。经 `MapSessionToExecution` 写入 `InternalExecutionConfig::detection.hardware` 和 `detection.engineering`，影响雷达方程、SNR、RCS 融合、方向图、检测裕量。 |
| `mission` | 波束方向、扫描限位、工作子模式与稳定方式。映射到 `detection.orientation`，影响扫描调度、波束指向、TAS/TWS/STT 等运行控制。 |
| `policy` | 波束控制、关联、跟踪、生命周期、IMM 策略。映射到 detection/association/tracking/lifecycle 各子配置，影响调度密度、关联代价、滤波、航迹确认与删除。 |
| `environment` | 初始化默认环境场景输入。传入 `EnvironmentService::UpdateModelConfig` / 构造函数，影响大气传播、植被散射、干扰源事实建模。 |
| `jamming_sensitivity_profile` | 干扰判定灵敏度语义档位。通过环境服务 `SetJammingSensitivityProfile` 生效，影响干扰是否被判定为存在以及后续抗干扰/决策输入。 |

### 1.2 `RadarHardwareConfig`

来源：`include/1q/airborne_radar/config/RadarHardwareConfig.h`

`RadarHardwareConfig` 当前只有一个聚合字段：

| 字段 | 影响功能 |
| --- | --- |
| `detection` | AR 探测链路配置总入口。初始化时复制到执行配置，并解析为内部 engineering 配置。 |

#### 1.2.1 `DetectionConfig`

| 字段 | 影响功能 |
| --- | --- |
| `enable_physics_detection` | 是否启用物理雷达方程检测链。启用后检测执行会更依赖发射机、接收机、天线、RCS 和传播输入计算 SNR/探测裕量。 |
| `transmitter` | 发射机工程参数，影响发射功率、频率、带宽、脉冲与发射链路损耗。 |
| `antenna` | 天线工程参数，影响主瓣增益、波束宽度、离轴方向图和扫描损失。 |
| `receiver` | 接收机工程参数，影响噪声底、接收链路损耗和最终 SNR。 |
| `detection_policy` | 检测判决门限，影响 CFAR 虚警目标与最低 SNR 门槛。 |
| `rcs_physics` | 目标 RCS 物理估计与经验 RCS 的融合参数。 |
| `min_detection_margin_db` | 最小探测裕量。检测执行中以 `margin >= min_detection_margin_db` 判定是否通过裕量门槛。值越低越容易检出，越高越保守。 |
| `pulse_count` | 脉冲积累数量。影响多脉冲检测概率、Swerling 起伏模型计算以及检测稳定性。 |

#### 1.2.2 `TransmitterConfig`

| 字段 | 影响功能 |
| --- | --- |
| `peak_power_w` | 峰值发射功率。功率越高，雷达方程中的发射功率项越大，远距目标 SNR 通常越高。 |
| `frequency_hz` | 工作频率。影响波长、传播和 RCS 物理估计；当 `rcs_physics.frequency_hz` 未单独设置时，业务上应保持两者一致。 |
| `bandwidth_hz` | 发射带宽。影响距离分辨率、噪声带宽相关计算和物理检测链。 |
| `pulse_width_s` | 脉宽。影响单脉冲能量与距离/时间相关检测特性。 |
| `prf_hz` | 脉冲重复频率。影响脉冲时序、积累和扫描/检测节奏。 |
| `transmit_loss_db` | 发射链路损耗。损耗越大，等效发射能量越低，SNR 越低。 |

#### 1.2.3 `AntennaConfig` 与 `AntennaPatternConfig`

| 字段 | 影响功能 |
| --- | --- |
| `main_beam_gain_db` | 主瓣峰值增益。影响主波束方向上的雷达方程增益。 |
| `nominal_az_beamwidth_deg` | 名义方位波束宽度。影响扫描步进、波束覆盖和默认波束尺寸。 |
| `nominal_el_beamwidth_deg` | 名义俯仰波束宽度。影响俯仰覆盖和默认波束尺寸。 |
| `pattern` | 离轴方向图参数集合。启用方向图时影响 off-boresight 增益衰减、旁瓣/后瓣响应和扫描损失。 |
| `enable_directional_pattern` | 是否启用离轴方向图评估。为 `false` 时主要使用主瓣/名义参数；为 `true` 时目标相对波束方向会影响有效增益。 |
| `pattern.model_type` | 主瓣近似模型：高斯、抛物线或余弦幂。影响离轴角到增益衰减的曲线形状。 |
| `pattern.max_sidelobe_level_db` | 最大旁瓣电平。影响旁瓣方向目标或干扰的可见程度。 |
| `pattern.backlobe_level_db` | 后瓣电平。影响背向目标或背向干扰响应。 |
| `pattern.scan_loss_coeff_db_per_deg2` | 扫描损失系数。离轴扫描角越大，损失按该系数增长。 |
| `pattern.max_scan_loss_db` | 扫描损失上限。限制扫描损失不会无限增大。 |
| `pattern.boresight_offset_deg` | 方向图相对安装轴偏置。影响方向图中心与机械安装轴之间的角度偏移。 |

#### 1.2.4 `ReceiverConfig` 与 `DetectionPolicyConfig`

| 字段 | 影响功能 |
| --- | --- |
| `receiver.noise_figure_db` | 接收机噪声系数。值越大，等效噪声越高，SNR 越低。 |
| `receiver.receive_loss_db` | 接收链路损耗。值越大，接收有效信号越低。 |
| `detection_policy.cfar_pfa` | CFAR 虚警概率目标。值越低，检测更保守；值越高，虚警容忍度更高。 |
| `detection_policy.min_snr_db` | 最小 SNR 门限。检测候选低于该门限时更难通过。 |

#### 1.2.5 `RcsPhysicsConfig`

| 字段 | 影响功能 |
| --- | --- |
| `enable_physical_rcs` | 是否启用物理 RCS 估计。关闭时主要依赖目标经验 RCS。 |
| `frequency_hz` | RCS 物理估计使用的频率。影响波长相关散射估计。 |
| `physics_mix_ratio` | 物理估计与经验值的混合比例。越高越依赖几何/物理散射模型。 |
| `cylinder_weight` | 圆柱散射模型权重。影响目标等效几何散射组合。 |
| `min_equivalent_radius_m` | 等效半径下界。防止目标几何过小导致非物理 RCS。 |
| `max_equivalent_radius_m` | 等效半径上界。防止目标几何过大导致异常 RCS。 |
| `min_rcs_m2` | RCS 裁剪下界。限制最终 RCS 不低于该值。 |
| `max_rcs_m2` | RCS 裁剪上界。限制最终 RCS 不超过该值。 |
| `bistatic_psi_offset_deg` | 双站角偏移补偿。影响双站/几何角相关的 RCS 修正。 |

#### 1.2.6 硬件相关 profile 枚举

这些枚举主要由 `RadarSessionConfigBuilder` 消费，用于填充详细字段；如果外部直接给 `RadarSessionConfig` 赋值，则枚举本身不会自动生效。

| 枚举 | 影响功能 |
| --- | --- |
| `RadarHardwareProfile` | 选择硬件能力档位，Builder 会设置发射功率、频率、天线增益、接收机噪声等。 |
| `DetectionIntentProfile` | 选择探测意图，Builder 会设置 `pulse_count`、`cfar_pfa`、`min_snr_db`、`min_detection_margin_db`。 |
| `RcsFusionProfile` | 选择 RCS 物理融合策略，Builder 会设置 `enable_physical_rcs` 和 `physics_mix_ratio`。 |
| `AntennaPatternProfile` | 选择方向图档位，Builder 会设置旁瓣、后瓣、扫描损失和方向图模型。 |
| `TrackingPolicyProfile` | 选择跟踪策略档位，Builder 会设置量测噪声、Kalman 后端和丢失衰减。 |
| `LifecyclePolicyProfile` | 选择航迹生命周期档位，Builder 会设置确认命中、丢失容忍和 lost 保留周期。 |

### 1.3 `RadarMissionConfig`

来源：`include/1q/airborne_radar/config/RadarMissionConfig.h`，其核心字段为 `orientation`。

| 字段 | 影响功能 |
| --- | --- |
| `orientation.mount_angles_deg` | 雷达相对机体的固定安装偏置。与平台姿态、扫描中心组合后影响实际波束指向。 |
| `orientation.scan_center_deg` | 基准指向方向，运行期可用补丁更新。影响 TWS/TAS 扫描中心和 STT 驻留方向。 |
| `orientation.mechanical_scan_limits_deg` | 机械扫描限位。约束可扫描的方位/俯仰物理范围。 |
| `orientation.electronic_scan_limits_deg` | 电子扫描限位。约束相控/电子扫描允许范围。 |
| `orientation.scan_start_position` | 二维扫描起始象限。影响扫描调度从哪个角点开始。 |
| `orientation.scan_sequence` | 二维扫描推进顺序。影响先方位后俯仰或其他扫描遍历方式。 |
| `orientation.work_sub_mode` | 工作子模式：待机、TAS、TWS、STT。影响扫描是否进行、采样密度和是否固定驻留。 |
| `orientation.commanded_beamwidth_enabled` | 是否启用指令态波束宽度覆盖。开启后 `commanded_beamwidth_deg` 会影响运行波束宽度。 |
| `orientation.commanded_beamwidth_deg` | 指令态方位/俯仰波束宽度。用于战术控制、ECCM、LPI 等运行态控制。 |
| `orientation.stabilization_mode` | 波束稳定方式。影响波束方向是否随机体、惯性或地面参考稳定。 |

### 1.4 `RadarPolicyConfig`

来源：`include/1q/airborne_radar/config/RadarPolicyConfig.h`

| 字段 | 影响功能 |
| --- | --- |
| `beam_control.pointing.default_scan_center_deg` | 默认扫描中心提示。影响波束控制的默认指向基线。 |
| `beam_control.pointing.nominal_beamwidth_deg` | 名义指令态波束宽度。影响波束调度和控制策略使用的基准宽度。 |
| `beam_control.scheduler.azimuth_step_count_hint` | 方位步进数提示。影响扫描调度密度；为 0 时通常由库内根据波束/扫描范围推导。 |
| `beam_control.scheduler.elevation_step_count_hint` | 俯仰步进数提示。影响俯仰扫描密度。 |
| `beam_control.scheduler.prefer_dense_tas_sampling` | 是否偏好更密 TAS 采样。影响 TAS 模式下空间采样密度。 |
| `association.unassigned_cost` | 未分配量测代价。影响数据关联优化中“不开新匹配”的代价权衡。 |
| `association.use_distance_gate_hint` | 是否启用外部距离门限 sigma 提示。关闭时由库内自适应计算门限。 |
| `association.distance_gate_sigma_hint` | 距离门限 sigma 提示值，仅在 `use_distance_gate_hint=true` 时参与关联门控。 |
| `tracking.enable_kalman_filter` | 是否启用 Kalman 滤波。影响航迹预测/更新是否走滤波链路。 |
| `tracking.kalman_measurement_noise_std` | 量测噪声标准差。影响关联距离度量、Kalman 更新协方差和航迹平滑程度。 |
| `tracking.kalman_update_backend` | Kalman 更新后端。影响 Joseph、UD、SRIF、EKF 等更新实现选择。 |
| `tracking.speed_decay_ratio_on_loss` | 丢失周期速度衰减系数。目标失配后影响航迹速度外推保守程度。 |
| `tracking.rcs_decay_ratio_on_loss` | 丢失周期 RCS 衰减系数。目标失配后影响航迹 RCS 维持或衰减。 |
| `lifecycle.confirm_hits` | tentative 航迹转 confirmed 所需命中数。越小确认越快，越大越稳健。 |
| `lifecycle.max_miss_before_lost` | 进入 lost 前允许的连续失配数。越大越能容忍短时遮挡或漏检。 |
| `lifecycle.max_lost_cycles` | lost 态最大保留周期。越大航迹保持时间越长，也可能保留更多过期航迹。 |
| `lifecycle.enable_imm_lifecycle` | 是否启用 IMM 生命周期路径。开启后内部会准备 IMM 模型噪声提示。 |
| `lifecycle.model_count_hint` | IMM 模型数提示。用于表达外部期望的模型数量；当前主要是语义提示。 |

### 1.5 `RadarEnvironmentConfig`

来源：`include/1q/airborne_radar/config/RadarEnvironmentConfig.h`，该类型是 `environment::EnvironmentDefaultConfig` 的别名。

| 字段 | 影响功能 |
| --- | --- |
| `scenario_config.atmospheric_physics.enable_physical_model` | 是否启用物理大气传播模型。影响环境服务是否使用气象输入推导传播/折射相关量。 |
| `scenario_config.atmospheric_physics.pressure_hpa` | 气压。影响有效地球半径因子、折射/传播估计。 |
| `scenario_config.atmospheric_physics.temperature_k` | 温度。影响折射率和传播估计。 |
| `scenario_config.atmospheric_physics.relative_humidity` | 相对湿度。影响湿度相关折射/传播估计。 |
| `scenario_config.atmospheric_context.has_simulation_unix_seconds` | 是否提供仿真时间。为 `true` 时 day-of-year 从时间戳推导。 |
| `scenario_config.atmospheric_context.simulation_unix_seconds` | Unix 秒级仿真时间戳。影响 day-of-year 推导。 |
| `scenario_config.atmospheric_context.solar_flux_f107a` | 平滑太阳流量指数。为空间天气上下文输入，供传播扩展使用。 |
| `scenario_config.atmospheric_context.solar_flux_f107` | 当日太阳流量指数。供空间天气/电离层传播扩展使用。 |
| `scenario_config.atmospheric_context.geomagnetic_ap` | 地磁活动指数。供空间天气/电离层传播扩展使用。 |
| `scenario_config.vegetation_scatter_physics.cover_profile` | 地表植被覆盖档位。影响植被散射、近地杂波和多径估计。 |
| `scenario_config.vegetation_scatter_physics.enable_physical_model` | 是否启用植被散射物理建模。 |
| `scenario_config.jammer_sources` | 干扰源事实列表。影响环境服务对干扰存在、干扰方向、干信比和后续抗干扰决策输入的建模。 |

`JammerEmitterState` 成员影响：

| 字段 | 影响功能 |
| --- | --- |
| `technique` | 干扰技术类型，影响干扰语义分类。 |
| `power_db` | 干扰功率估计，影响干扰是否超过灵敏度门限。 |
| `js_db` | 干扰与信号比估计，影响干扰严重度。 |
| `has_direction_deg` | 是否提供干扰来向。为 `false` 时方向相关判断不能使用外部来向。 |
| `azimuth_deg` | 干扰来向方位。影响是否落入主瓣/旁瓣等方向判断。 |
| `elevation_deg` | 干扰来向俯仰。影响方向相关干扰判断。 |
| `angular_span_deg` | 干扰角域宽度。影响干扰覆盖区域。 |
| `confidence` | 干扰事实置信度。影响环境服务对干扰事实的可信程度。 |

### 1.6 `RadarRuntimeConfigPatch`

来源：`include/1q/airborne_radar/config/RadarRuntimeConfigPatch.h`

补丁应用顺序是整域覆盖后叶子覆盖；叶子字段具有最终优先级。非有限角度会被拒绝，补丁整体不生效。

| 字段 | 影响功能 |
| --- | --- |
| `has_mission` / `mission` | 整块覆盖任务域，影响方向、扫描、工作子模式和波束宽度等 mission 字段。 |
| `has_policy` / `policy` | 整块覆盖策略域，并重新解析 tracking/lifecycle/association 工程配置。 |
| `has_environment_runtime_config` / `environment_runtime_config` | 更新环境运行期配置，可更新 `scenario_config` 和 `jamming_sensitivity_profile`。 |
| `has_work_sub_mode` / `work_sub_mode` | 快捷更新工作子模式。 |
| `has_scan_center_deg` / `scan_center_deg` | 快捷更新基准扫描中心。 |
| `has_dwell_center_deg` / `dwell_center_deg` | 更新运行期驻留偏移；最终 pipeline session 会将其叠加到 `scan_center_deg`。 |
| `has_commanded_beamwidth_deg` / `commanded_beamwidth_deg` | 快捷更新指令态波束宽度。 |
| `has_commanded_beamwidth_enabled` / `commanded_beamwidth_enabled` | 快捷更新指令态波束宽度覆盖开关。 |

## 2. EOS 光电传感器配置

### 2.1 初始化聚合：`session::EosSessionConfig`

来源：`include/1q/electro_optical_sensor/config/EosSessionConfig.h`

| 字段 | 影响功能 |
| --- | --- |
| `hardware` | 光学硬件规格。映射到 `EosPipelineConfig`，影响波段、口径、焦距和后续辐射/探测计算。 |
| `mission` | 工作模式、视场、扫描、帧率和视轴。映射到 pipeline 的工作模式与扫描/成像参数。 |
| `policy` | 探测与杂散光策略。影响 SNR 门限、探测灵敏度、可见光参考辐照度、遮光罩抑制。 |
| `environment` | 环境模型语义输入。通过 `BuildModelConfigFromScenario` 转为 pipeline 环境模型参数。 |

### 2.2 `EosHardwareConfig`

| 字段 | 影响功能 |
| --- | --- |
| `wavelength_lower_um` | 工作波段下限。影响红外/可见光辐射传输与能量积分范围。 |
| `wavelength_upper_um` | 工作波段上限。与下限共同定义传感器响应波段。 |
| `optical_aperture_m` | 光学口径。影响入射能量、分辨能力和探测灵敏度。 |
| `focal_length_m` | 焦距。影响成像几何、视场投影和目标成像尺度。 |

### 2.3 `EosMissionConfig`

| 字段 | 影响功能 |
| --- | --- |
| `work_mode` | 工作模式：红外、可见光或融合。影响 pipeline 选择使用的探测通道。 |
| `horizontal_fov_deg` | 水平视场角。影响可观测区域和目标是否落入画幅。 |
| `vertical_fov_deg` | 垂直视场角。影响俯仰覆盖范围。 |
| `scan_rate_deg_per_sec` | 扫描角速度。影响扫描覆盖节奏和目标驻留时间。 |
| `frame_rate_hz` | 帧率。影响时间采样、输出帧频和检测更新频率。 |
| `scan_start_az_deg` | 扫描起始方位。影响扫描窗口起点。 |
| `scan_end_az_deg` | 扫描结束方位。影响扫描窗口终点。 |
| `scan_center_el_deg` | 扫描中心俯仰。影响视场俯仰中心。 |
| `boresight_depression_deg` | 视轴下俯角。影响传感器朝向和目标进入视场的几何关系。 |

### 2.4 `EosPolicyConfig`

`EosDetectionPolicyConfig` 支持 profile 模式与详细参数模式。`use_profile_defaults=true` 时，`profile` 会映射到内部默认参数；`false` 时直接使用详细字段。

| 字段 | 影响功能 |
| --- | --- |
| `detection.profile` | 探测策略档位。保守档提高 SNR 门限和灵敏度要求；激进档降低门限、提高检出倾向。 |
| `detection.use_profile_defaults` | 是否使用 profile 默认映射。为 `true` 时忽略下面三个详细参数。 |
| `detection.minimum_snr_db` | 最小检测 SNR 门限。越高越保守。 |
| `detection.detection_sensitivity_w` | 探测灵敏度。影响目标辐射功率达到可检出的门槛。 |
| `detection.visible_reference_irradiance_w_m2` | 可见光参考辐照度。影响可见光通道目标/背景对比基准。 |
| `stray_light.profile` | 杂散光防护档位。控制是否启用遮光罩以及抑制参数。 |
| `stray_light.use_profile_defaults` | 是否使用杂散光 profile 默认映射。为 `true` 时忽略下面详细参数。 |
| `stray_light.enable_straylight_filter` | 是否启用杂散光抑制。影响太阳/强光源附近的背景抑制。 |
| `stray_light.hood_inner_half_angle_deg` | 遮光罩内半角。影响强光进入抑制区的几何判定。 |
| `stray_light.hood_outer_half_angle_deg` | 遮光罩外半角。影响杂散光影响范围。 |
| `stray_light.hood_min_suppression_ratio` | 最小抑制比。决定弱抑制场景下的下限。 |
| `stray_light.hood_max_suppression_ratio` | 最大抑制比。决定强抑制场景下的上限。 |

### 2.5 `EosEnvironmentConfig`

来源：`include/1q/electro_optical_sensor/config/EosEnvironmentConfig.h`，真实结构位于 `environment/EosEnvironmentConfig.h`。

| 字段 | 影响功能 |
| --- | --- |
| `scenario_config.model_type` | 环境模型类型。`kSimplified` 使用简化环境参数；`kAdvanced` 允许高度、风速、云量等高级环境输入驱动 pipeline。 |
| `scenario_config.preset` | 高层环境预设。映射到辐射传输模型、气溶胶密度因子和湍流因子。 |
| `scenario_config.has_custom_overrides` | 是否使用显式自定义覆盖。为 `true` 时覆盖 preset 映射结果。 |
| `scenario_config.custom_overrides.radiative_transfer_model` | 辐射传输模型。影响大气透过率、路径辐射和目标辐射衰减。 |
| `scenario_config.custom_overrides.aerosol_density_factor` | 气溶胶密度因子。越高，大气衰减和散射影响通常越强。 |
| `scenario_config.custom_overrides.turbulence_factor` | 湍流因子。影响高级环境模型下的扰动强度。 |
| `scenario_config.custom_overrides.enable_optical_countermeasure_extension` | 是否启用光电对抗扩展语义。当前模型配置会保存该值，供扩展环境/对抗链路消费。 |

### 2.6 `EosRuntimeConfigPatch`

来源：`include/1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h`

| 字段 | 影响功能 |
| --- | --- |
| `has_mission` / `mission` | 整块覆盖 mission，影响工作模式、视场、扫描、帧率、视轴。 |
| `has_policy` / `policy` | 整块覆盖 policy，影响探测和杂散光策略。 |
| `has_environment` / `environment` | 更新环境场景配置；EOS 环境补丁当前支持 `has_scenario_config` / `scenario_config`。 |
| `has_work_mode` / `work_mode` | 快捷更新工作模式。 |
| `has_scan_rate_deg_per_sec` / `scan_rate_deg_per_sec` | 快捷更新扫描角速度。 |
| `has_frame_rate_hz` / `frame_rate_hz` | 快捷更新帧率。 |

## 3. ESR 电子侦察配置

### 3.1 初始化聚合：`session::EsrSessionConfig`

来源：`include/1q/electronic_surveillance_radar/config/EsrSessionConfig.h`

| 字段 | 影响功能 |
| --- | --- |
| `hardware` | 接收机频段、灵敏度、波束宽度、扫描范围和安装偏置。经 `ResolveEsrSessionConfig` 映射到 pipeline 和 runtime 配置。 |
| `mission` | 开关机、工作模式和扫描策略。影响传感器启停、扫描率、扫描窗口和统计检测参数修正。 |
| `policy` | 截获/探测策略。影响最小截获 SNR、虚警概率、脉冲积累、门限缩放和统计检测开关。 |
| `environment` | 环境场景语义输入。映射到 ESR 环境服务模型配置。 |

### 3.2 `EsrHardwareConfig`

| 字段 | 影响功能 |
| --- | --- |
| `receiver_band_lower_hz` | 接收频段下限。与上限有效且上限更大时启用固定接收窗口。 |
| `receiver_band_upper_hz` | 接收频段上限。影响可截获辐射源频率范围。 |
| `receiver_sensitivity_w` | 接收机灵敏度。有效正值会映射为 pipeline 检测噪声底。 |
| `integrated_receive_loss_db` | 系统综合接收损耗。有效正值写入 runtime 配置，影响接收链路等效损耗。 |
| `beam_az_width_deg` | 方位波束宽度。有效正值用作扫描方位步进。 |
| `beam_el_width_deg` | 俯仰波束宽度。有效正值用作扫描俯仰步进。 |
| `az_scan_range_deg` | 方位扫描范围。未使用显式边界时，用于从扫描中心推导起止方位。 |
| `el_scan_range_deg` | 俯仰扫描范围。未使用显式边界时，用于从扫描中心推导起止俯仰。 |
| `antenna_mount_az_deg` | 天线中心方位安装偏置。扫描中心/边界会减去该偏置后进入 pipeline。 |
| `antenna_mount_el_deg` | 天线中心俯仰安装偏置。扫描中心/边界会减去该偏置后进入 pipeline。 |

### 3.3 `EsrMissionConfig`

| 字段 | 影响功能 |
| --- | --- |
| `power_on` | 传感器开关状态。映射为 runtime `sensor_enabled`。 |
| `work_mode` | 工作模式。`kHgesm` 会提高脉冲积累并降低门限缩放，`kRwr` 会减少积累并提高门限缩放，`kEsm` 保持默认。 |
| `scan.scan_center_az_deg` | 扫描中心方位。未启用显式边界时结合硬件方位扫描范围推导起止方位。 |
| `scan.scan_center_el_deg` | 扫描中心俯仰。未启用显式边界时结合硬件俯仰扫描范围推导起止俯仰。 |
| `scan.scan_rate_hz` | 扫描数据率。有限且大于 0 时写入 runtime；否则回退为 1.0 Hz。 |
| `scan.scan_start_position` | 扫描起始位置。映射为 pipeline 扫描起点。 |
| `scan.scan_sequence` | 二维扫描推进顺序。映射为 pipeline 扫描顺序。 |
| `scan.use_explicit_scan_bounds` | 是否使用显式扫描起止角。为 `true` 且四个边界有限时，扫描中心和硬件扫描范围不再决定边界。 |
| `scan.scan_start_az_deg` | 显式扫描起始方位。使用时会减去天线安装方位偏置，并进行起止归一化。 |
| `scan.scan_end_az_deg` | 显式扫描结束方位。使用时会减去天线安装方位偏置，并进行起止归一化。 |
| `scan.scan_start_el_deg` | 显式扫描起始俯仰。使用时会减去天线安装俯仰偏置，并进行起止归一化。 |
| `scan.scan_end_el_deg` | 显式扫描结束俯仰。使用时会减去天线安装俯仰偏置，并进行起止归一化。 |

### 3.4 `EsrPolicyConfig`

`EsrDetectionPolicyConfig` 支持 profile 模式与详细参数模式。

| 字段 | 影响功能 |
| --- | --- |
| `detection.profile` | 探测策略档位。保守档将最小截获 SNR 设为 10 dB，高灵敏档设为 3 dB，均衡档使用默认值。 |
| `detection.use_profile_defaults` | 是否使用 profile 默认映射。为 `true` 时忽略详细参数。 |
| `detection.min_detect_snr_db` | 详细模式下的最小截获 SNR 门限。越高越保守。 |
| `detection.pfa` | 详细模式下的期望虚警概率。影响统计检测门限。 |
| `detection.pulse_count` | 详细模式下的脉冲积累数量。工作模式还会对该值进行倍增或折减。 |
| `detection.threshold_scale` | 详细模式下的门限缩放系数。工作模式会进一步修正该值，并限制不低于 0.1。 |
| `detection.enable_statistical_detection` | 是否启用统计检测模型。关闭后截获链路不使用统计检测扩展。 |

### 3.5 `EsrEnvironmentConfig`

来源：`include/1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h`，真实结构位于 `environment/EsrEnvironmentConfig.h`。

| 字段 | 影响功能 |
| --- | --- |
| `scenario_config.preset` | 环境预设语义：标准、低杂波、高杂波、强干扰。传入环境服务模型，影响环境快照和 pipeline 可消费的环境态。 |
| `scenario_config.atmospheric_physics.enable_physical_model` | 是否启用物理大气传播模型。 |
| `scenario_config.atmospheric_physics.pressure_hpa` | 气压。供传播/折射模型使用。 |
| `scenario_config.atmospheric_physics.temperature_k` | 温度。供传播/折射模型使用。 |
| `scenario_config.atmospheric_physics.relative_humidity` | 相对湿度。供传播/折射模型使用。 |
| `scenario_config.atmospheric_context.has_k_factor` | 是否显式提供有效地球半径因子。 |
| `scenario_config.atmospheric_context.k_factor` | 显式有效地球半径因子。影响传播几何近似。 |
| `scenario_config.atmospheric_context.has_day_of_year` | 是否显式提供年积日。 |
| `scenario_config.atmospheric_context.day_of_year` | 年积日。影响季节/太阳几何相关传播扩展。 |
| `scenario_config.atmospheric_context.solar_flux_f107a` | 平滑太阳流量指数。供空间天气扩展使用。 |
| `scenario_config.atmospheric_context.solar_flux_f107` | 当日太阳流量指数。供空间天气扩展使用。 |
| `scenario_config.atmospheric_context.geomagnetic_ap` | 地磁活动指数。供空间天气扩展使用。 |

### 3.6 `EsrRuntimeConfigPatch`

来源：`include/1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h`

补丁应用顺序是整域覆盖后叶子覆盖；叶子字段具有最终优先级。环境 preset 不支持运行期热更新，`has_preset=true` 会被拒绝。

| 字段 | 影响功能 |
| --- | --- |
| `has_mission` / `mission` | 整块覆盖任务域，更新传感器开关、扫描率，并按工作模式修正统计检测参数。 |
| `has_policy` / `policy` | 整块覆盖策略域，重新应用探测策略。 |
| `has_environment_runtime_config` / `environment_runtime_config` | 更新环境模型叶子字段；支持 atmospheric physics/context，不支持 preset 热更新。 |
| `has_sensor_enabled` / `sensor_enabled` | 快捷更新传感器开关状态。 |
| `has_work_mode` / `work_mode` | 快捷更新工作模式，并按模式修正统计检测参数。 |
| `has_scan_rate_hz` / `scan_rate_hz` | 快捷更新扫描数据率。必须有限且大于 0，否则补丁拒绝。 |
| `has_scan_start_position` / `scan_start_position` | 快捷更新扫描起始位置。 |
| `has_scan_sequence` / `scan_sequence` | 快捷更新扫描推进顺序。 |
| `has_scan_center_az_deg` / `scan_center_az_deg` | 快捷更新扫描中心方位，保持原扫描半宽不变并重算起止方位。 |
| `has_scan_center_el_deg` / `scan_center_el_deg` | 快捷更新扫描中心俯仰，保持原扫描半宽不变并重算起止俯仰。 |
| `has_use_explicit_scan_bounds` / `use_explicit_scan_bounds` | 是否切换显式扫描边界。开启时必须同时提供四个有限边界值，否则补丁拒绝。 |
| `has_scan_start_az_deg` / `scan_start_az_deg` | 显式扫描起始方位。与边界模式一起使用。 |
| `has_scan_end_az_deg` / `scan_end_az_deg` | 显式扫描结束方位。与边界模式一起使用。 |
| `has_scan_start_el_deg` / `scan_start_el_deg` | 显式扫描起始俯仰。与边界模式一起使用。 |
| `has_scan_end_el_deg` / `scan_end_el_deg` | 显式扫描结束俯仰。与边界模式一起使用。 |

## 4. 使用建议

- 初始化时优先构造 `*SessionConfig` 的四域字段；只有需要运行期热更新时才构造 `*RuntimeConfigPatch`。
- 使用 profile 的字段，应注意 `use_profile_defaults=true` 会忽略详细参数；要直接指定详细阈值或抑制参数，必须关闭 profile 默认映射。
- 环境 `config` 头在 EOS/ESR 中主要是别名门面；字段真源在对应 `environment/*EnvironmentConfig.h`。
- AR 的 `jamming_sensitivity_profile` 当前位于 `RadarSessionConfig` 顶层，而环境场景事实位于 `environment.scenario_config`。
- ESR 的环境 preset 是初始化语义，运行期补丁显式禁止热更新 preset。
