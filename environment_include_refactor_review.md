# include/environment 对外设计重构审查报告

## 审查范围

- `airborne_radar`
- `electronic_surveillance_radar`
- `electro_optical_sensor`

审查目标：

1. 来自外部世界、且库内无法可靠自造的量，必须有输入通道。
2. 输入通道应为高层语义输入，不应把中间物理参数和调参项直接摊给用户。

据此，本报告采用三类判断：

- `应保留/新增的外部输入`：外部事实或高层语义，库内不能可靠自造。
- `应下沉为库内派生`：可由高层输入、平台状态、目标状态、时间空间上下文推导。
- `当前暴露层级不当`：公开 include 已把中间物理量/调参项直接暴露给用户。

## 总结结论

### 结论概览

- `airborne_radar` 最接近目标方向。它已经有 `EnvironmentScenarioConfig -> EnvironmentModelConfig` 的表面分层，但公开层仍残留较多“物理细项直接暴露”，尤其是大气高级上下文和植被散射参数。
- `electronic_surveillance_radar` 偏离目标最明显。当前公开 `environment` API 直接要求用户输入传播损耗分项、杂波噪声、阈值、路径长度、仰角等中间物理参数，违反“高层语义输入”原则。
- `electro_optical_sensor` 处于另一种偏离：会话配置层已经偏高层，但 `environment` include 仍直接暴露 `radiative_transfer_model`、`aerosol_density_factor`、`turbulence_factor` 等内部模型参数，形成双入口且层次冲突。

### 模块排序

1. `airborne_radar`：结构方向基本正确，但需要继续去物理细项化。
2. `electro_optical_sensor`：高层入口已有雏形，但公开环境接口需要收口。
3. `electronic_surveillance_radar`：需要整体重构公开 environment 语义模型。

## 一、airborne_radar 审查

### 1. 当前设计优点

- 已经把对外默认配置落在 `EnvironmentScenarioConfig`，并通过 `BuildModelConfigFromScenario` 转成内部模型配置，这个方向是正确的。[EnvironmentConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/environment/EnvironmentConfig.h#L114) [EnvironmentConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/environment/EnvironmentConfig.h#L134)
- Session 组合根以场景输入驱动环境服务，而不是要求调用方直接构造内部环境服务配置，这符合“高层输入，内部映射”的要求。[RadarSessionCompositionRoot.cpp](/Users/aurora/Code/1q/src/airborne_radar/session/RadarSessionCompositionRoot.cpp#L25)
- 干扰源事实输入与冻结后的派生事实输出做了区分，这是公开边界设计里最成熟的一块。[EnvironmentConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/environment/EnvironmentConfig.h#L23) [EnvironmentConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/environment/EnvironmentConfig.h#L43)
- 运行期更新也优先围绕 `EnvironmentScenarioConfig`，而不是要求用户直接 patch 内部快照字段。[RadarRuntimeConfigBuilder.h](/Users/aurora/Code/1q/include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h#L71)

### 2. 与原则冲突的点

#### 2.1 大气高级上下文仍直接暴露为物理细项

`AtmosphericDerivedContext` 直接把 `k_factor`、`day_of_year`、`solar_flux_f107a`、`solar_flux_f107`、`geomagnetic_ap` 暴露在公开 include 中。[EnvironmentConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/environment/EnvironmentConfig.h#L75)

问题：

- 这些量虽然文档说是“高层时间/空间天气上下文”，但从用户视角仍然是专业物理输入，不是业务语义输入。
- 其中至少一部分可由更高层语义派生，例如时间戳、地理位置、气候/空间天气数据源句柄、天气等级/模板等。
- 当前接口让用户直接填这些值，实际是在把环境模型求值输入暴露出去。

建议：

- 对外改为更高层结构，例如 `AtmosphereObservationInput` / `WeatherContextInput`。
- 包含建议语义：`timestamp_utc`、`geo_region` 或 `platform_lla`、`weather_source_kind`、`weather_profile`、可选 `surface_weather_observation`。
- `k_factor/F107/AP/day_of_year` 进入库内派生层，最多放在内部 `src/` 配置。

#### 2.2 植被散射参数仍然是中间物理模型参数

`VegetationScatterPhysicsConfig` 直接暴露 `leaf_size_m`、`dielectric_constant_real`、`leaf_count`、`canopy_radius_m`、`canopy_height_m`。[EnvironmentConfig.h](/Users/aurora/Code/1q/include/1q/airborne_radar/environment/EnvironmentConfig.h#L91)

问题：

- 这些不是用户天然拥有的“外部世界事实”，而是散射模型入参。
- `PropagationModel` 内部又基于这些参数进一步求入射角、散射角、phase matrix 和 clutter multiplier，说明它们处在模型中间层，而非真正边界层。[PropagationModel.cpp](/Users/aurora/Code/1q/src/airborne_radar/environment/PropagationModel.cpp#L32) [PropagationModel.cpp](/Users/aurora/Code/1q/src/airborne_radar/environment/PropagationModel.cpp#L48)

建议：

- 对外改为高层地表/植被语义输入，如 `LandCoverType`、`VegetationDensityLevel`、`CanopyClass`、`SeasonType`、`WetnessLevel`。
- 若必须支持专家用户，保留“外部环境服务扩展点”而不是默认 include 暴露物理细项。

#### 2.3 干扰方向缺失时的派生逻辑过于拍脑袋

当 `has_direction_deg=false` 时，库根据 `technique/js_db/power_db/confidence` 估算方向、旁瓣进入、频率重叠和 PRF 锁定风险。[EnvironmentService.cpp](/Users/aurora/Code/1q/src/airborne_radar/environment/EnvironmentService.cpp#L29) [EnvironmentService.cpp](/Users/aurora/Code/1q/src/airborne_radar/environment/EnvironmentService.cpp#L66) [EnvironmentService.cpp](/Users/aurora/Code/1q/src/airborne_radar/environment/EnvironmentService.cpp#L93)

判断：

- `frequency_overlap_ratio`、`prf_lock_risk`、`in_sidelobe` 作为派生输出是合理的。
- 但“来向角”本身属于外部事实，库内若无观测基础，不应强行推造。

建议：

- 保留 `azimuth/elevation` 作为可选外部输入是对的。
- 当缺失时，输出中应明确为“未知/未提供”，而不是伪造估计方向。
- 这比当前做法更符合“外部无法可靠自造的量必须有输入通道；库内无法可靠自造时不要假造”。

### 3. 结论

`airborne_radar` 应保留现有“场景输入 -> 内部模型”的总体结构，但需要把以下公开类型从 include 表面收回：

- `AtmosphericDerivedContext`
- `VegetationScatterPhysicsConfig`
- 面向外部可直接构造的 `EnvironmentModelConfigBuilder`

`EnvironmentModelConfigBuilder` 目前仍公开可构造内部模型配置，削弱了分层边界。[EnvironmentModelConfigBuilder.h](/Users/aurora/Code/1q/include/1q/airborne_radar/environment/EnvironmentModelConfigBuilder.h#L17)

## 二、electronic_surveillance_radar 审查

### 1. 当前设计的主要问题

该模块公开 `environment` 基本等同于公开内部求值参数。

典型表现：

- `EsrAtmosphericPhysicsConfig` 直接要求用户提供 `frequency_hz`、`path_length_m`、`radar_altitude_m`、`target_altitude_m`、`elevation_deg` 等传播计算入参。[EsrEnvironmentConfig.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h#L17)
- `EsrEnvironmentSceneState` 直接要求用户输入 `base_propagation_loss_db`、`atmospheric_attenuation_db`、`terrain_reflection_db`、`clutter_noise_w`、`spectrum_occupancy_ratio`。[EsrEnvironmentTypes.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h#L45)
- Builder 进一步把这些中间量显式包装成对外 API。[EsrEnvironmentSceneBuilder.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentSceneBuilder.h#L22)
- 默认配置和 runtime patch 也继续暴露 `default_clutter_noise_w` 与 `jamming_detection_threshold_w`。[EsrEnvironmentConfig.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentConfig.h#L37) [EsrEnvironmentConfigBuilder.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentConfigBuilder.h#L27)

这与目标原则直接冲突。

### 2. 哪些量不应作为公开输入

以下字段应视为内部模型参数，不应直接出现在对外 include：

- `base_propagation_loss_db`
- `atmospheric_attenuation_db`
- `terrain_reflection_db`
- `clutter_noise_w`
- `default_clutter_noise_w`
- `jamming_detection_threshold_w`
- `path_length_m`
- `elevation_deg`
- 在默认环境配置里直接暴露的 `frequency_hz`

依据：

- `BuildSnapshot` 会把这些量直接送入传播/干扰判定过程，本质是求值器中间层，而不是边界层事实。[EsrEnvironmentService.cpp](/Users/aurora/Code/1q/src/electronic_surveillance_radar/environment/EsrEnvironmentService.cpp#L37)
- `propagation_loss_db` 是由场景项和物理损耗聚合得出，说明这些字段本身就属于内部计算分项。[EsrEnvironmentService.cpp](/Users/aurora/Code/1q/src/electronic_surveillance_radar/environment/EsrEnvironmentService.cpp#L67)

### 3. 哪些量应该保留为外部输入

应保留/新增：

- 周期级电磁环境事实：干扰源列表、可观测频谱活动、外部辐射源真值。
- 平台与目标/辐射源几何关系所需的外部事实。
- 时间、空间、天气等可驱动物理传播的高层上下文。

当前已经相对合理的一项是 `EsrJammerSource`，因为中心频率、带宽、功率、置信度、欺骗风险更接近外部电子战事实。[EsrEnvironmentTypes.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/environment/EsrEnvironmentTypes.h#L28)

但仍需调整：

- `deception_risk` 如果代表“库内分类/评估后的风险”，则不应作为外部字段；若代表外部情报先验，则应改名明确成 `deception_confidence` 或 `deception_likelihood_hint`。
- `active` 是否保留取决于语义。如果它表示外部观测到的开机状态，可保留；如果只是内部过滤开关，应去掉。

### 4. 现有高层入口反而是正确方向

Session 层已经有高层环境策略 `EsrEnvironmentPolicyConfig`，只暴露预设，而不是物理细项。[EsrEnvironmentPolicyConfig.h](/Users/aurora/Code/1q/include/1q/electronic_surveillance_radar/config/EsrEnvironmentPolicyConfig.h#L24)

并且 resolver 也是把高层 `preset` 映射成内部模型配置。[EsrSessionConfigResolver.cpp](/Users/aurora/Code/1q/src/electronic_surveillance_radar/session/EsrSessionConfigResolver.cpp#L168) [EsrSessionConfigResolver.cpp](/Users/aurora/Code/1q/src/electronic_surveillance_radar/session/EsrSessionConfigResolver.cpp#L225)

这说明该模块已经存在正确方向，只是公开 `environment` include 没有跟上。

### 5. 重构建议

建议把 ESR 公开 `environment` 重构成两层：

#### 对外公开层

- `EsrEnvironmentObservation`
  - 时间/地理区域/平台位置
  - 可观测频谱占用等级或带宽段活动摘要
  - 干扰源事实列表
  - 可选天气/传播上下文
- `EsrEnvironmentPolicy`
  - `preset`
  - 可选 `environment_mode`

#### 内部实现层

- `EsrPropagationModelInputs`
- `EsrClutterModelInputs`
- `EsrInterferenceAssessmentInputs`

这些内部类型应下沉到 `src/`，不再经由 `include/1q/.../environment` 暴露。

### 6. 结论

`electronic_surveillance_radar` 需要做结构性重构，不是小修小补。当前 include 表面的环境设计整体上把“模型参数输入接口”误当成了“用户环境输入接口”。

## 三、electro_optical_sensor 审查

### 1. 当前设计的矛盾

EOS 的 Session 配置层已经是高层语义：

- `EosEnvironmentPolicyConfig` 只暴露 `model_type + preset`。[EosEnvironmentPolicyConfig.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/config/EosEnvironmentPolicyConfig.h#L25)
- `BuildEosPipelineConfig` 会把预设映射为 `radiative_transfer_model/aerosol_density_factor/turbulence_factor`。[EosPipelineConfigMapper.cpp](/Users/aurora/Code/1q/src/electro_optical_sensor/runtime/EosPipelineConfigMapper.cpp#L77) [EosPipelineConfigMapper.cpp](/Users/aurora/Code/1q/src/electro_optical_sensor/runtime/EosPipelineConfigMapper.cpp#L111)

但公开 `environment` include 又把这些内部模型参数重新暴露了出来：

- `radiative_transfer_model`
- `aerosol_density_factor`
- `turbulence_factor`

见：[EosEnvironmentConfig.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/environment/EosEnvironmentConfig.h#L23)

这造成：

- 同一模块有两个入口层次。
- 高层 Session 入口和低层 environment 入口表达的是同一件事，但抽象层不一致。
- 用户可以绕开高层策略，直接调底层模型细项。

### 2. 哪些量应该被视为内部派生量

`ResolveEnvironmentFactors` 已明确说明：

- 输入高层上下文：`platform_altitude_m`、`cloud_coverage_ratio`、`wind_speed_mps`
- 结合 `base_aerosol_density_factor`、`base_turbulence_factor`
- 派生出 `aerosol_density_factor`、`turbulence_factor`、`path_radiance_scale_bias`

见：[EosEnvironmentModel.cpp](/Users/aurora/Code/1q/src/electro_optical_sensor/environment/EosEnvironmentModel.cpp#L22)

因此：

- `aerosol_density_factor`
- `turbulence_factor`
- `path_radiance_scale_bias`

本质上都属于库内环境模型输出或中间参数，而不是优先暴露给用户的环境输入。

### 3. 哪些外部输入是合理的

`EosCycleInput` 中以下字段是合理的外部世界输入：

- `solar_altitude_deg`
- `solar_azimuth_deg`
- `solar_irradiance_w_m2`
- `cloud_coverage_ratio`
- `ambient_wind_speed_mps`
- `day_night_type`
- `background_temperature_k`
- 目标辐射/外观特征

见：[EosCycleInput.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/model/EosCycleInput.h#L45)

其中需要特别指出：

- `atmospheric_transmittance` 不应作为默认公开输入继续保留。

原因：

- 它与 `aerosol_density_factor/turbulence_factor/radiative_transfer_model` 一样，都是偏中间的模型量。
- 库已具备从更高层上下文计算环境因子的能力，再让用户直接给 `atmospheric_transmittance` 会形成“双轨输入”，导致语义冲突。

### 4. 当前公开 environment 接口的问题

- `EosEnvironmentConfigBuilder` 直接允许外部设置内部模型参数。[EosEnvironmentConfigBuilder.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/environment/EosEnvironmentConfigBuilder.h#L17)
- `EosEnvironmentRuntimeConfigPatch` 继续允许运行期 patch `radiative_transfer_model/aerosol_density_factor/turbulence_factor`。[EosEnvironmentRuntimeConfigPatch.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/environment/EosEnvironmentRuntimeConfigPatch.h#L18)

这两个接口都应收口。

### 5. 重构建议

建议 EOS 公开 `environment` 只保留两类能力：

#### 高层环境事实输入

- `EosEnvironmentObservation`
  - 太阳几何
  - 云量
  - 风速
  - 背景温度
  - 可选能见度/湿度/气溶胶等级
  - 时间地点

#### 高层环境策略输入

- `EosEnvironmentPolicyConfig`
  - `preset`
  - `model_type`

而以下内容应转入内部：

- `RadiativeTransferModel`
- `aerosol_density_factor`
- `turbulence_factor`
- `path_radiance_scale_bias`

如果必须支持专家级覆盖，建议通过扩展服务接口实现，而不是默认对所有 include 用户开放。当前 `IEosEnvironmentService` 本身就是更适合承载专家自定义环境模型的机制。[IEosEnvironmentService.h](/Users/aurora/Code/1q/include/1q/electro_optical_sensor/environment/IEosEnvironmentService.h#L15)

### 6. 结论

`electro_optical_sensor` 不需要像 ESR 那样整体翻修，但需要明确“高层会话输入是唯一主入口”，并把公开 environment 中的内部模型细项下沉。

## 四、跨模块统一问题

### 1. 三个模块公开层级不一致

- `airborne_radar`：偏场景事实输入。
- `electronic_surveillance_radar`：偏模型参数输入。
- `electro_optical_sensor`：高层策略与低层模型双入口并存。

这会带来：

- 用户学习成本高。
- 多模块联用时环境建模语义不统一。
- 后续扩展难以形成统一 API 风格。

### 2. 建议统一的对外 environment 分层

建议三个模块都统一成下面的公开设计模式：

#### `Observation`

外部世界事实输入。只放库内不能可靠自造的量，例如：

- 时间
- 地点/平台位置
- 外部天气观测
- 外部频谱/干扰事实
- 目标/辐射源/背景事实

#### `Policy`

高层策略输入。只放用户可理解的语义控制，例如：

- `preset`
- `model_type`
- `mode`
- `quality_level`

#### `Snapshot`

库内冻结后的派生结果输出，例如：

- 传播损耗
- 杂波估计
- 干扰检测结果
- 环境风险等级

#### `InternalModelInputs`

仅放在 `src/` 或非默认扩展层：

- 气溶胶因子
- 湍流因子
- 传播分项损耗
- 杂波基线噪声
- 各类 threshold/scale/bias
- 植被散射几何/介电参数

## 五、优先级建议

### P0：必须先改

- ESR 公开 `environment` 中直接暴露的传播损耗分项、杂波噪声、阈值、路径长度、仰角等模型参数。
- EOS 公开 `environment` 中直接暴露的 `radiative_transfer_model/aerosol_density_factor/turbulence_factor`。

### P1：建议紧随其后

- AR 的 `AtmosphericDerivedContext` 高级物理字段外露。
- AR 的 `VegetationScatterPhysicsConfig` 外露。
- AR 的 `EnvironmentModelConfigBuilder` 公开存在。

### P2：设计质量改进

- AR 缺失干扰方向时不应伪造方向，应改为“未知”语义输出。
- EOS 去掉 `atmospheric_transmittance` 这类中间量与高层环境事实并列输入的做法。
- ESR 重新命名 `deception_risk`，明确它是外部事实先验还是内部评估结果。

## 六、建议的重构落地顺序

1. 先定义统一术语：`Observation / Policy / Snapshot / InternalModelInputs`。
2. 先改 ESR，因为问题最重且收益最大。
3. 再改 EOS，消除高层策略入口与低层环境入口的重复表达。
4. 最后收紧 AR，把残留物理细项下沉。
5. 在三个模块中统一规则：默认 include 只暴露 `Observation + Policy + Snapshot + Service Interface`。

## 最终结论

基于“外部事实必须有输入通道，但输入通道必须保持高层语义”的原则：

- `airborne_radar`：方向基本正确，属于“继续收口公开物理细项”。
- `electro_optical_sensor`：已有正确高层入口，属于“去掉重复且过低层的公开 environment 入口”。
- `electronic_surveillance_radar`：当前公开 environment 基本是内部模型参数面板，属于“需要整体重构公开语义模型”。

