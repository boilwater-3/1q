# 配置输入清单（面向需求/甲方审阅）

> 目的：对齐三模块配置颗粒度，明确本库**当前公开暴露**的配置项（初始化 + 运行期）。
> 说明：仅基于 `include/1q/*/config` 与其别名指向的公开头文件，不包含 `src/` 内部实现细节。

## 1. Airborne Radar（AR）配置输入

### 1.1 初始化输入入口
- 入口类型：`airborne_radar::session::RadarSessionConfig`
- 头文件：
  - `include/1q/airborne_radar/config/RadarSessionConfig.h`
  - `include/1q/airborne_radar/config/RadarHardwareConfig.h`
  - `include/1q/airborne_radar/config/RadarMissionConfig.h`
  - `include/1q/airborne_radar/config/RadarPolicyConfig.h`
  - `include/1q/airborne_radar/config/RadarEnvironmentConfig.h`

### 1.2 初始化字段（按四域）

#### A) `hardware`（`RadarHardwareConfig`）
- `detection`（`DetectionConfig`）
  - `enable_physics_detection`
  - `transmitter`
    - `peak_power_w`
    - `frequency_hz`
    - `bandwidth_hz`
    - `pulse_width_s`
    - `prf_hz`
    - `transmit_loss_db`
  - `antenna`
    - `main_beam_gain_db`
    - `nominal_az_beamwidth_deg`
    - `nominal_el_beamwidth_deg`
    - `enable_directional_pattern`
    - `pattern`
      - `model_type`
      - `max_sidelobe_level_db`
      - `backlobe_level_db`
      - `scan_loss_coeff_db_per_deg2`
      - `max_scan_loss_db`
      - `boresight_offset_deg`
  - `receiver`
    - `noise_figure_db`
    - `receive_loss_db`
  - `detection_policy`
    - `cfar_pfa`
    - `min_snr_db`
  - `rcs_physics`
    - `enable_physical_rcs`
    - `frequency_hz`
    - `physics_mix_ratio`
    - `cylinder_weight`
    - `min_equivalent_radius_m`
    - `max_equivalent_radius_m`
    - `min_rcs_m2`
    - `max_rcs_m2`
    - `bistatic_psi_offset_deg`
  - `min_detection_margin_db`
  - `pulse_count`
  - `swerling_model`

#### B) `mission`（`RadarMissionConfig`）
- `orientation`（`RadarOrientationConfig`）
  - `mount_angles_deg`
  - `scan_center_deg`
  - `mechanical_scan_limits_deg`
  - `electronic_scan_limits_deg`
  - `scan_start_position`
  - `scan_sequence`
  - `work_sub_mode`
  - `commanded_beamwidth_enabled`
  - `commanded_beamwidth_deg`
  - `stabilization_mode`

#### C) `policy`（`RadarPolicyConfig`）
- `beam_control`
  - `pointing.default_scan_center_deg`
  - `pointing.nominal_beamwidth_deg`
  - `scheduler.azimuth_step_count_hint`
  - `scheduler.elevation_step_count_hint`
  - `scheduler.prefer_dense_tas_sampling`
- `association`
  - `unassigned_cost`
  - `use_distance_gate_hint`
  - `distance_gate_sigma_hint`
- `tracking`
  - `enable_kalman_filter`
  - `kalman_measurement_noise_std`
  - `kalman_update_backend`
  - `speed_decay_ratio_on_loss`
  - `rcs_decay_ratio_on_loss`
- `lifecycle`
  - `confirm_hits`
  - `max_miss_before_lost`
  - `max_lost_cycles`
  - `enable_imm_lifecycle`
- `imm`
  - `enable_imm_lifecycle`
  - `model_count_hint`

#### D) `environment`（`RadarEnvironmentConfig` -> `EnvironmentDefaultConfig`）
- `scenario_config`
  - `atmospheric_physics`（`AtmosphericObservation`）
    - `enable_physical_model`
    - `pressure_hpa`
    - `temperature_k`
    - `relative_humidity`
  - `atmospheric_context`（AR 专用上下文）
    - `has_simulation_unix_seconds`
    - `simulation_unix_seconds`
    - `solar_flux_f107a`
    - `solar_flux_f107`
    - `geomagnetic_ap`
  - `vegetation_scatter_physics`
    - `cover_profile`
    - `enable_physical_model`
  - `jammer_sources[]`
    - `technique`
    - `power_db`
    - `js_db`
    - `has_direction_deg`
    - `azimuth_deg`
    - `elevation_deg`
    - `angular_span_deg`
    - `confidence`

#### E) 独立初始化字段
- `jamming_sensitivity_profile`（位于 `RadarSessionConfig` 顶层）

### 1.3 运行期输入入口
- 入口类型：`airborne_radar::config::RadarRuntimeConfigPatch`
- 头文件：
  - `include/1q/airborne_radar/config/RadarRuntimeConfigPatch.h`
  - `include/1q/airborne_radar/environment/EnvironmentRuntimeConfigPatch.h`

### 1.4 运行期可变字段
- 整域：`has_mission/mission`、`has_policy/policy`、`has_environment_runtime_config/environment_runtime_config`
- 叶子：
  - `has_work_sub_mode/work_sub_mode`
  - `has_scan_center_deg/scan_center_deg`
  - `has_dwell_center_deg/dwell_center_deg`
  - `has_commanded_beamwidth_deg/commanded_beamwidth_deg`
  - `has_commanded_beamwidth_enabled/commanded_beamwidth_enabled`
- 环境运行期补丁（`EnvironmentRuntimeConfigPatch`）
  - `has_scenario_config/scenario_config`
  - `has_jamming_sensitivity_profile/jamming_sensitivity_profile`

---

## 2. Electro Optical Sensor（EOS）配置输入

### 2.1 初始化输入入口
- 入口类型：`electro_optical_sensor::session::EosSessionConfig`
- 头文件：
  - `include/1q/electro_optical_sensor/config/EosSessionConfig.h`
  - `include/1q/electro_optical_sensor/config/EosHardwareConfig.h`
  - `include/1q/electro_optical_sensor/config/EosMissionConfig.h`
  - `include/1q/electro_optical_sensor/config/EosPolicyConfig.h`
  - `include/1q/electro_optical_sensor/config/EosEnvironmentConfig.h`

### 2.2 初始化字段（按四域）

#### A) `hardware`（`EosHardwareConfig`）
- `wavelength_lower_um`
- `wavelength_upper_um`
- `optical_aperture_m`
- `focal_length_m`

#### B) `mission`（`EosMissionConfig`）
- `work_mode`
- `horizontal_fov_deg`
- `vertical_fov_deg`
- `scan_rate_deg_per_sec`
- `frame_rate_hz`
- `scan_start_az_deg`
- `scan_end_az_deg`
- `scan_center_el_deg`
- `boresight_depression_deg`

#### C) `policy`（`EosPolicyConfig`）
- `detection`
  - `profile`
  - `use_profile_defaults`
  - `minimum_snr_db`
  - `detection_sensitivity_w`
  - `visible_reference_irradiance_w_m2`
- `stray_light`
  - `profile`
  - `use_profile_defaults`
  - `enable_straylight_filter`
  - `hood_inner_half_angle_deg`
  - `hood_outer_half_angle_deg`
  - `hood_min_suppression_ratio`
  - `hood_max_suppression_ratio`

#### D) `environment`（`EosEnvironmentConfig` -> `EosEnvironmentDefaultConfig`）
- `scenario_config`
  - `model_type`
  - `preset`
  - `has_custom_overrides`
  - `custom_overrides`
    - `radiative_transfer_model`
    - `aerosol_density_factor`
    - `turbulence_factor`
    - `enable_optical_countermeasure_extension`

### 2.3 运行期输入入口
- 入口类型：`electro_optical_sensor::session::EosRuntimeConfigPatch`
- 头文件：
  - `include/1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h`
  - `include/1q/electro_optical_sensor/environment/EosEnvironmentRuntimeConfigPatch.h`

### 2.4 运行期可变字段
- 整域：`has_mission/mission`、`has_policy/policy`、`has_environment/environment`
- 叶子：
  - `has_work_mode/work_mode`
  - `has_scan_rate_deg_per_sec/scan_rate_deg_per_sec`
  - `has_frame_rate_hz/frame_rate_hz`
- 环境运行期补丁（白名单）：
  - `has_model_type/model_type`
  - `has_radiative_transfer_model/radiative_transfer_model`
  - `has_aerosol_density_factor/aerosol_density_factor`
  - `has_turbulence_factor/turbulence_factor`
  - `has_enable_optical_countermeasure_extension/enable_optical_countermeasure_extension`

---

## 3. Electronic Surveillance Radar（ESR）配置输入

### 3.1 初始化输入入口
- 入口类型：`electronic_surveillance_radar::session::EsrSessionConfig`
- 头文件：
  - `include/1q/electronic_surveillance_radar/config/EsrSessionConfig.h`
  - `include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h`
  - `include/1q/electronic_surveillance_radar/config/EsrMissionConfig.h`
  - `include/1q/electronic_surveillance_radar/config/EsrPolicyConfig.h`
  - `include/1q/electronic_surveillance_radar/config/EsrEnvironmentConfig.h`

### 3.2 初始化字段（按四域）

#### A) `hardware`（`EsrHardwareConfig`）
- `receiver_band_lower_hz`
- `receiver_band_upper_hz`
- `receiver_sensitivity_w`
- `integrated_receive_loss_db`
- `beam_az_width_deg`
- `beam_el_width_deg`
- `az_scan_range_deg`
- `el_scan_range_deg`
- `antenna_mount_az_deg`
- `antenna_mount_el_deg`

#### B) `mission`（`EsrMissionConfig`）
- `power_on`
- `work_mode`
- `scan`（`EsrScanPolicyConfig`）
  - `scan_center_az_deg`
  - `scan_center_el_deg`
  - `scan_rate_hz`
  - `scan_start_position`
  - `scan_sequence`
  - `use_explicit_scan_bounds`
  - `scan_start_az_deg`
  - `scan_end_az_deg`
  - `scan_start_el_deg`
  - `scan_end_el_deg`

#### C) `policy`（`EsrPolicyConfig`）
- `detection`（`EsrDetectionPolicyConfig`）
  - `profile`
  - `use_profile_defaults`
  - `min_detect_snr_db`
  - `pfa`
  - `pulse_count`
  - `threshold_scale`
  - `enable_statistical_detection`

#### D) `environment`（`EsrEnvironmentConfig` -> `EsrEnvironmentDefaultConfig`）
- `scenario_config`
  - `preset`
  - `atmospheric_physics`（`AtmosphericObservation`）
    - `enable_physical_model`
    - `pressure_hpa`
    - `temperature_k`
    - `relative_humidity`
  - `atmospheric_context`（`SpaceWeatherContext`）
    - `has_k_factor`
    - `k_factor`
    - `has_day_of_year`
    - `day_of_year`
    - `solar_flux_f107a`
    - `solar_flux_f107`
    - `geomagnetic_ap`

### 3.3 运行期输入入口
- 入口类型：`electronic_surveillance_radar::session::EsrRuntimeConfigPatch`
- 头文件：
  - `include/1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h`
  - `include/1q/electronic_surveillance_radar/environment/EsrEnvironmentRuntimeConfigPatch.h`

### 3.4 运行期可变字段
- 整域：`has_mission/mission`、`has_policy/policy`、`has_environment_runtime_config/environment_runtime_config`
- 叶子：
  - `has_sensor_enabled/sensor_enabled`
  - `has_work_mode/work_mode`
  - `has_scan_rate_hz/scan_rate_hz`
  - `has_scan_start_position/scan_start_position`
  - `has_scan_sequence/scan_sequence`
  - `has_scan_center_az_deg/scan_center_az_deg`
  - `has_scan_center_el_deg/scan_center_el_deg`
  - `has_use_explicit_scan_bounds/use_explicit_scan_bounds`
  - `has_scan_start_az_deg/scan_start_az_deg`
  - `has_scan_end_az_deg/scan_end_az_deg`
  - `has_scan_start_el_deg/scan_start_el_deg`
  - `has_scan_end_el_deg/scan_end_el_deg`
- 环境运行期补丁（白名单）
  - `has_atmospheric_physics/atmospheric_physics`
  - `has_atmospheric_context/atmospheric_context`
  - `has_preset/preset`：字段仍在类型中，但当前语义为**已弃用**，若置 `true` 将被 reject（不支持运行时 preset 热更新）。

---

> 如需用于甲方评审的“变更版”清单，可在此文档基础上追加两列：
> - `是否运行期可改`（Y/N）
> - `来源层级`（硬件固有 / 任务语义 / 策略语义 / 环境事实）

## 4. 参数注释（甲方审阅版）

> 使用方式：按上文字段名检索本节对应条目即可。  
> 标记约定：
> - `has_*`：是否启用某个补丁字段；`false` 表示该字段不参与本次更新。
> - `*_deg`：角度（度）。`*_hz`：频率（Hz）。`*_w`：功率（瓦）。

### 4.1 AR 参数注释

#### AR / hardware.detection
- `enable_physics_detection`：是否启用物理雷达方程检测链。
- `peak_power_w`：发射峰值功率，越大通常探测距离越远。
- `frequency_hz`：工作频率，影响传播损耗与目标散射特性。
- `bandwidth_hz`：发射带宽，影响距离分辨能力。
- `pulse_width_s`：脉宽，影响能量积累与分辨特性。
- `prf_hz`：脉冲重复频率，影响测速与模糊特性。
- `transmit_loss_db`：发射链路损耗，越大表示硬件损耗越重。
- `main_beam_gain_db`：主瓣增益，越大方向性越强。
- `nominal_az_beamwidth_deg / nominal_el_beamwidth_deg`：名义波束宽度。
- `enable_directional_pattern`：是否启用离轴方向图衰减模型。
- `model_type`：主瓣近似模型类型（高斯/抛物线/余弦幂）。
- `max_sidelobe_level_db`：最大旁瓣电平（越低越有利于抗干扰）。
- `backlobe_level_db`：后瓣电平。
- `scan_loss_coeff_db_per_deg2`：扫描离轴损失系数。
- `max_scan_loss_db`：扫描损失上限。
- `boresight_offset_deg`：天线方向图相对安装轴偏置。
- `noise_figure_db`：接收机噪声系数。
- `receive_loss_db`：接收链路损耗。
- `cfar_pfa`：CFAR 虚警概率目标。
- `min_snr_db`：最低检测信噪比门限。
- `enable_physical_rcs`：是否启用物理 RCS 估计。
- `physics_mix_ratio`：物理 RCS 与经验值融合权重。
- `cylinder_weight`：圆柱散射模型权重。
- `min/max_equivalent_radius_m`：等效半径裁剪范围。
- `min/max_rcs_m2`：RCS 裁剪范围。
- `bistatic_psi_offset_deg`：双站角补偿项。
- `min_detection_margin_db`：最小检测裕量。
- `pulse_count`：脉冲积累数。
- `swerling_model`：目标 RCS 起伏统计模型。

#### AR / mission.orientation
- `mount_angles_deg`：雷达相对机体安装偏角。
- `scan_center_deg`：扫描中心基准指向。
- `mechanical/electronic_scan_limits_deg`：机械/电子扫描边界。
- `scan_start_position`：扫描起始象限。
- `scan_sequence`：扫描推进顺序（先方位或先俯仰）。
- `work_sub_mode`：工作子模式（待机/TAS/TWS/STT）。
- `commanded_beamwidth_enabled`：是否启用指令态波束宽度覆盖。
- `commanded_beamwidth_deg`：指令态瞬时波束宽度。
- `stabilization_mode`：波束稳定策略（随体/惯性/对地）。

#### AR / policy
- `pointing.default_scan_center_deg`：默认扫描中心。
- `pointing.nominal_beamwidth_deg`：名义波束宽度。
- `scheduler.azimuth/elevation_step_count_hint`：扫描步数提示。
- `scheduler.prefer_dense_tas_sampling`：是否偏好 TAS 高密度采样。
- `association.unassigned_cost`：关联中“未匹配”惩罚代价。
- `association.use_distance_gate_hint`：是否使用外部距离门限提示。
- `association.distance_gate_sigma_hint`：距离门限 sigma 提示值。
- `tracking.enable_kalman_filter`：是否启用 Kalman 跟踪。
- `tracking.kalman_measurement_noise_std`：量测噪声标准差。
- `tracking.kalman_update_backend`：Kalman 更新后端实现。
- `tracking.speed/rcs_decay_ratio_on_loss`：失配时速度/RCS 衰减比例。
- `lifecycle.confirm_hits`：确认航迹所需命中次数。
- `lifecycle.max_miss_before_lost`：变为 lost 前可容忍连续失配次数。
- `lifecycle.max_lost_cycles`：lost 状态最大保留周期。
- `lifecycle.enable_imm_lifecycle`：是否启用 IMM 生命周期路径。
- `imm.enable_imm_lifecycle`：IMM 路径总开关。
- `imm.model_count_hint`：IMM 模型数提示。

#### AR / environment
- `atmospheric_physics.*`：基础气象观测输入（压强/温度/湿度等）。
- `atmospheric_context.*`：时间与空间天气上下文输入。
- `vegetation_scatter_physics.cover_profile`：植被场景档位。
- `vegetation_scatter_physics.enable_physical_model`：是否启用植被散射建模。
- `jammer_sources[]`：场景干扰源事实列表。
- `jamming_sensitivity_profile`：干扰判定灵敏度档位。

### 4.2 EOS 参数注释

#### EOS / hardware
- `wavelength_lower_um / wavelength_upper_um`：工作波段范围。
- `optical_aperture_m`：光学口径（影响进光能力）。
- `focal_length_m`：焦距（影响视场/放大关系）。

#### EOS / mission
- `work_mode`：工作模式（红外/可见光/融合等）。
- `horizontal_fov_deg / vertical_fov_deg`：水平/垂直视场角。
- `scan_rate_deg_per_sec`：扫描角速度。
- `frame_rate_hz`：图像帧率。
- `scan_start_az_deg / scan_end_az_deg`：方位扫描起止角。
- `scan_center_el_deg`：俯仰扫描中心角。
- `boresight_depression_deg`：视轴下俯角。

#### EOS / policy
- `detection.profile`：检测策略档位。
- `detection.use_profile_defaults`：是否使用档位默认参数。
- `minimum_snr_db`：最小检测信噪比门限。
- `detection_sensitivity_w`：检测灵敏度。
- `visible_reference_irradiance_w_m2`：可见光参考辐照度。
- `stray_light.profile`：杂散光抑制档位。
- `stray_light.use_profile_defaults`：是否使用杂散光档位默认参数。
- `enable_straylight_filter`：是否启用杂散光过滤。
- `hood_inner/outer_half_angle_deg`：遮光罩内/外半角。
- `hood_min/max_suppression_ratio`：杂散光抑制比例边界。

#### EOS / environment
- `model_type`：环境模型复杂度选择。
- `preset`：环境预设（标准/潮湿/沙尘等）。
- `has_custom_overrides`：是否启用自定义覆盖。
- `custom_overrides.radiative_transfer_model`：辐射传输模型类型。
- `custom_overrides.aerosol_density_factor`：气溶胶密度系数。
- `custom_overrides.turbulence_factor`：湍流系数。
- `custom_overrides.enable_optical_countermeasure_extension`：是否启用光学对抗扩展。

#### EOS / 运行期补丁
- `has_mission / has_policy / has_environment`：是否整域覆盖对应配置。
- `has_work_mode`：是否更新工作模式。
- `has_scan_rate_deg_per_sec`：是否更新扫描速度。
- `has_frame_rate_hz`：是否更新帧率。
- `environment` 内 `has_*` 字段：环境白名单叶子更新开关。

### 4.3 ESR 参数注释

#### ESR / hardware
- `receiver_band_lower_hz / receiver_band_upper_hz`：接收频段上下限。
- `receiver_sensitivity_w`：接收灵敏度。
- `integrated_receive_loss_db`：综合接收损耗。
- `beam_az_width_deg / beam_el_width_deg`：方位/俯仰波束宽度。
- `az_scan_range_deg / el_scan_range_deg`：方位/俯仰扫描范围。
- `antenna_mount_az_deg / antenna_mount_el_deg`：天线安装偏角。

#### ESR / mission.scan
- `power_on`：设备是否开机。
- `work_mode`：ESM/HGESM/RWR 工作模式。
- `scan_center_az_deg / scan_center_el_deg`：扫描中心角。
- `scan_rate_hz`：扫描数据率。
- `scan_start_position`：扫描起始象限。
- `scan_sequence`：扫描推进顺序。
- `use_explicit_scan_bounds`：是否使用显式扫描边界。
- `scan_start/end_az_deg`：方位扫描边界。
- `scan_start/end_el_deg`：俯仰扫描边界。

#### ESR / policy.detection
- `profile`：检测策略档位（保守/均衡/灵敏）。
- `use_profile_defaults`：是否使用档位默认参数。
- `min_detect_snr_db`：最小截获信噪比门限。
- `pfa`：虚警概率目标。
- `pulse_count`：脉冲积累数。
- `threshold_scale`：门限缩放系数。
- `enable_statistical_detection`：是否启用统计检测模型。

#### ESR / environment
- `scenario_config.preset`：环境预设档位（标准/低杂波/高杂波/强干扰）。
- `atmospheric_physics.*`：基础气象观测输入。
- `atmospheric_context.*`：空间天气与时间上下文（含 k 因子、年积日等）。

#### ESR / 运行期补丁
- `has_mission / has_policy / has_environment_runtime_config`：整域覆盖开关。
- `has_sensor_enabled`：是否更新传感器使能。
- `has_work_mode`：是否更新工作模式。
- `has_scan_rate_hz`：是否更新扫描率。
- `has_scan_start_position / has_scan_sequence`：是否更新扫描起点/顺序。
- `has_scan_center_az_deg / has_scan_center_el_deg`：是否更新扫描中心。
- `has_use_explicit_scan_bounds`：是否切换显式边界模式。
- `has_scan_start/end_az_deg`、`has_scan_start/end_el_deg`：是否更新边界角值。
- `environment_runtime_config.has_atmospheric_physics/context`：环境叶子字段热更新。
- `environment_runtime_config.has_preset`：**已弃用，不支持热更新；置 true 会被 reject**。
