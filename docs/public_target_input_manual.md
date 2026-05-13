# 对外公开目标输入说明手册

本文说明三个外部公开模块的单周期目标输入接口，覆盖 `include/1q/.../session` 下的 `*CycleInput`、`*SceneTypes`、`*ExternalInputAdapter`、`*CycleInputBuilder` 和输入校验接口。

当前三个模块的单周期输入外层已经统一为：

- `cycle_index`：当前周期号。
- `dt_sec`：当前周期步长，单位秒。
- `platform_pose`：本周期平台位置、速度和姿态。
- `scene`：本周期目标或辐射源列表，是本文重点。
- `environment`：本周期环境事实输入。

`scene` 的语义因模块而不同：AR 输入雷达目标，EOS 输入光电可见目标，ESR 输入电子侦察辐射源。

## 1. 公共位姿输入

三个模块都复用或等价复用 `oneq::foundation::PoseState`。

来源：`include/1q/foundation/pose_types.h`

| 字段 | 说明 | 影响功能 |
| --- | --- | --- |
| `position_m.x/y/z` | 局部坐标位置，单位 m。 | 作为平台或目标/辐射源的几何基准，影响相对距离、方位、俯仰、传播损耗和扫描判定。 |
| `velocity_mps.x/y/z` | 局部坐标速度，单位 m/s。 | 影响相对速度、目标运动、跟踪预测或辐射源运动状态。 |
| `attitude_deg.yaw/pitch/roll` | 欧拉角姿态，单位 deg。 | 影响局部坐标系朝向、目标视线角、发射波束或传感器指向。 |

外部适配器通常接收 ECEF 位置/速度和 Body->ENU 姿态，再转换为模块局部坐标输入。

## 2. AR 机载雷达目标输入

### 2.1 单周期输入：`RadarCycleInput`

来源：`include/1q/airborne_radar/session/RadarCycleInput.h`

| 字段 | 说明 | 影响功能 |
| --- | --- | --- |
| `cycle_index` | 当前周期号。 | 用于周期记录、回放和输出对齐。 |
| `dt_sec` | 当前周期步长。 | 影响跟踪预测、状态推进和输入校验；非有限或 `<=0` 会产生 error 级校验问题。 |
| `platform_pose` | 当前周期雷达平台位姿。 | 影响雷达局部参考、运动补偿、目标相对速度和控制/环境上下文。 |
| `scene` | `RadarSceneTargetList`。 | 当前周期雷达目标事实列表，直接驱动检测、关联、跟踪和输出。 |
| `environment` | `RadarEnvironmentInput`。 | 当前周期大气、地表和干扰事实，影响传播、干扰判定和抗干扰逻辑。 |

### 2.2 目标类型：`RadarSceneTarget`

来源：`include/1q/airborne_radar/session/RadarSceneTypes.h`

AR 的目标输入采用雷达局部坐标系。调用方可以直接填局部笛卡尔坐标，也可以通过外部适配器从 ECEF 目标运动学转换。

| 字段 | 说明 | 影响功能 |
| --- | --- | --- |
| `external_target_id` | 外部目标标识，`0` 表示未知/未提供。 | 用于跨周期关联、输出航迹回填和调试定位；重复非 0 ID 会被校验为 error，0 ID 会产生 info。 |
| `velocity_x/y/z` | 雷达局部坐标速度分量，单位 m/s。 | 输入到目标运动和跟踪链路；外部适配器会将 ECEF 目标速度转到雷达局部坐标后扣除平台速度，得到相对速度。 |
| `rcs` | 目标雷达散射截面积，单位 m^2。 | 影响雷达方程、SNR、物理 RCS 融合和检测概率；负值会产生 warning。 |
| `range_m` | 目标到雷达的斜距，单位 m。 | 影响传播损耗、雷达方程和检测门限。若 `range_m <= 0`，必须提供非零笛卡尔位置，否则校验为 error。 |
| `position_x/y/z` | 雷达局部笛卡尔坐标，单位 m。 | 用于几何位置、距离回填、关联和航迹状态。 |
| `target_swerling_type` | 目标起伏模型编号。 | 影响多脉冲检测概率中使用的 Swerling 模型。 |

### 2.3 构造工具：`RadarSceneTargetUtils`

来源：`include/1q/airborne_radar/session/RadarSceneTargetUtils.h`

| 接口 | 说明 | 适用场景 |
| --- | --- | --- |
| `MakeSceneTarget(...)` | 使用三维位置、速度、RCS、Swerling 类型构造目标，并写入斜距。 | 已有雷达局部三维目标事实。 |
| `MakeGroundSceneTarget(...)` | 构造 `z=0` 的地面目标。 | 地面目标、车辆、地表散射体等。 |
| `MakeAirSceneTarget(...)` | 构造三维空中目标。 | 飞机、无人机、空中目标。 |
| `NormalizeSceneTargetGeometry(...)` | 当 `range_m <= 0` 时，从位置范数回填斜距。 | 手写目标列表后统一补齐几何派生量。 |

### 2.4 外部输入适配：`RadarExternalInputAdapter`

来源：`include/1q/airborne_radar/session/RadarExternalInputAdapter.h`

| 类型/接口 | 字段或行为 | 说明 |
| --- | --- | --- |
| `RadarExternalPoseInput` | `platform_position_ecef_m` | 平台 ECEF 位置。适配器将其转为 LLA 参考原点；输出 `platform_pose.position_m` 当前为雷达局部原点。 |
| `RadarExternalPoseInput` | `platform_velocity_mps` | 平台 ECEF 速度。适配器转 ENU 后再转雷达局部速度。 |
| `RadarExternalPoseInput` | `platform_attitude_deg` | 平台姿态，Body->ENU。输出平台姿态保存为 foundation 欧拉角。 |
| `RadarExternalPoseInput` | `radar_mount_angles_deg` | 雷达安装角，Body->Radar。与平台姿态通过矩阵复合为雷达局部姿态。 |
| `TargetExternalKinematics` | `target_position_ecef_m` | 目标 ECEF 位置。转换到雷达局部 `position_x/y/z` 并计算 `range_m`。 |
| `TargetExternalKinematics` | `target_velocity_mps` | 目标 ECEF 速度。转换到雷达局部后扣除平台局部速度，写入相对速度。 |
| `TargetExternalKinematics` | `rcs` | 目标 RCS，原样写入 `RadarSceneTarget::rcs`。 |
| `TargetExternalKinematics` | `swerling_type` | 目标起伏模型编号，原样写入。 |
| `TryMakeRadarPoseFromExternalKinematics(...)` | 两步模式第一步。 | 生成局部参考系与平台位姿。 |
| `TryMakeTargetFromExternalKinematics(...)` | 两步模式第二步。 | 使用第一步参考系把单个外部目标转为 `RadarSceneTarget`。 |

### 2.5 一步构建器：`RadarCycleInputBuilder`

来源：`include/1q/airborne_radar/session/RadarCycleInputBuilder.h`

`RadarCycleInputBuilder::Build(...)` 封装平台转换和目标转换，输出可直接传入 `RadarSession::Step()` 的 `RadarCycleInput`。

注意事项：

- `output == nullptr` 时返回 `false`。
- 构建器会把 `cycle_index` 置为 `0`，调用方若需要真实周期号，应在构建后覆盖。
- 当前实现会用目标数组索引作为 `external_target_id`，不是外部系统原始目标 ID。
- 有环境重载版本可同时写入完整 `RadarEnvironmentInput`。

### 2.6 AR 输入校验

来源：`include/1q/airborne_radar/session/RadarInputValidation.h`

重点校验项：

- `dt_sec` 必须有限且 `>0`。
- `platform_pose` 所有位置、速度、姿态数值必须有限。
- 目标数值字段必须有限。
- 每个目标必须有正 `range_m` 或非零笛卡尔位置。
- 非 0 `external_target_id` 不得重复。
- `rcs < 0` 是 warning，不是 error。
- 环境气压/温度必须为正，相对湿度和干扰置信度必须在 `[0, 1]`。

## 3. EOS 光电目标输入

### 3.1 单周期输入：`EosCycleInput`

来源：`include/1q/electro_optical_sensor/session/EosCycleInput.h`

| 字段 | 说明 | 影响功能 |
| --- | --- | --- |
| `cycle_index` | 当前周期号。 | 用于输出、回放和周期对齐。 |
| `dt_sec` | 当前周期步长。 | 影响扫描相位推进和输入校验。 |
| `platform_pose` | 平台位姿。 | 影响外部坐标转换后的相对视线角、平台高度和环境模型输入。 |
| `scene` | `EosSceneTargetList`。 | 当前周期光电目标列表，驱动可见光/红外探测。 |
| `environment` | `EosEnvironmentInput`。 | 当前周期太阳、云量、风速、背景温度和昼夜事实输入。 |

### 3.2 目标外观：`EosTargetAppearance`

来源：`include/1q/electro_optical_sensor/session/EosSceneTypes.h`

| 字段 | 说明 | 影响功能 |
| --- | --- | --- |
| `apparent_temperature_k` | 目标等效温度，单位 K。 | 影响红外辐射强度；必须 `>0`。 |
| `emissivity` | 红外辐射效率，范围 `[0, 1]`。 | 影响红外通道目标辐射能量。 |
| `reflectance` | 可见光反射率，范围 `[0, 1]`。 | 影响可见光通道目标反射能量。 |
| `projected_area_m2` | 等效投影面积，单位 m^2。 | 影响目标入瞳能量；必须 `>0`。 |

`emissivity + reflectance > 1` 会被校验为能量平衡不一致。

### 3.3 目标类型：`EosSceneTarget`

EOS 的目标输入是传感器视线空间中的目标状态，而不是三维局部笛卡尔坐标。

| 字段 | 说明 | 影响功能 |
| --- | --- | --- |
| `target_id` | 目标标识。 | 输出检测记录会回填该 ID；`0` 会被校验为 error。 |
| `range_m` | 目标斜距，单位 m。 | 影响辐射衰减、目标能量和检测概率；必须 `>0`。 |
| `azimuth_deg` | 目标方位角，单位 deg。 | 影响是否落入当前扫描视场、杂散光角距离和输出检测角度。 |
| `elevation_deg` | 目标仰角，单位 deg。 | 影响视场判定、地平/天空背景和输出检测角度。 |
| `appearance` | 目标外观与辐射参数。 | 影响红外/可见光能量计算和探测判决。 |

### 3.4 外部输入适配：`EosExternalInputAdapter`

来源：`include/1q/electro_optical_sensor/session/EosExternalInputAdapter.h`

| 类型/接口 | 字段或行为 | 说明 |
| --- | --- | --- |
| `EosExternalPoseInput` | `platform_position_ecef_m` | 平台 ECEF 位置。构建器会先转 LLA，作为局部 ENU 原点。 |
| `EosExternalPoseInput` | `platform_velocity_mps` | 平台 ECEF 速度。适配器转换为 EOS 局部速度。 |
| `EosExternalPoseInput` | `platform_attitude_deg` | 平台姿态，Body->ENU。也作为 EOS 局部参考姿态。 |
| `EosExternalTargetInput.position_frame` | `kEcef` 或 `kLla`。 | 指定目标位置来自 ECEF 还是 WGS84 LLA。 |
| `EosExternalTargetInput.target_position_ecef_m` | 目标 ECEF 坐标。 | `position_frame == kEcef` 时使用。 |
| `EosExternalTargetInput.target_position_lla_deg_m` | 目标 LLA 坐标。 | `position_frame == kLla` 时使用。 |
| `EosExternalTargetInput.appearance` | 目标外观。 | 转换后原样写入 `EosSceneTarget::appearance`。 |
| `TryMakeEosPoseFromExternalKinematics(...)` | 平台转换。 | 输出平台局部位姿。 |
| `TryMakeEosSceneTargetFromExternalInput(...)` | 目标转换。 | 计算目标相对平台的 `range_m / azimuth_deg / elevation_deg`。 |

适配失败状态：

- `kNullOutput`：输出指针为空。
- `kCoordinateTransformFail`：ECEF/LLA/ENU 转换失败。
- `kDegenerateGeometry`：目标与平台几何距离过小，无法形成有效视线角。

### 3.5 一步构建器：`EosCycleInputBuilder`

来源：`include/1q/electro_optical_sensor/session/EosCycleInputBuilder.h`

注意事项：

- 构建器会根据平台 ECEF 位置生成局部参考原点。
- 构建器会把 `cycle_index` 置为 `0`，调用方可在构建后覆盖。
- 当前实现用目标数组索引作为 `target_id`；如果外部系统需要稳定 ID，可使用两步 adapter 自行传入 ID，或构建后覆盖。
- 有环境重载版本可同时写入完整 `EosEnvironmentInput`。

### 3.6 EOS 输入校验

来源：`include/1q/electro_optical_sensor/session/EosInputValidation.h`

重点校验项：

- `dt_sec` 必须有限且 `>0`。
- 平台位姿数值必须有限。
- 环境太阳辐照度不能为负，云量必须在 `[0, 1]`，风速不能为负，背景温度必须 `>0`。
- 太阳高度角必须在 `[-90, 90]`，昼夜类型应与太阳高度角一致。
- `target_id` 不能为 `0`。
- `range_m`、目标温度、投影面积必须 `>0`。
- `emissivity`、`reflectance` 必须在 `[0, 1]`，且二者之和不能超过 1。

## 4. ESR 电子侦察辐射源输入

### 4.1 单周期输入：`EsrCycleInput`

来源：`include/1q/electronic_surveillance_radar/session/EsrCycleInput.h`

| 字段 | 说明 | 影响功能 |
| --- | --- | --- |
| `cycle_index` | 当前周期号。 | 用于输出、回放和环境冻结上下文。 |
| `dt_sec` | 当前周期步长。 | 影响运行周期推进和输入校验。 |
| `platform_pose` | 侦察平台姿态与运动状态。 | 影响接收几何、相对位置和拦截处理上下文。 |
| `scene` | `EsrSceneEmitterList`。 | 当前周期场景辐射源列表，驱动截获、识别和威胁评估。 |
| `environment` | `EsrEnvironmentInput`。 | 当前周期传播、杂波、频谱占用和干扰事实。 |

### 4.2 辐射源波束：`EsrEmitterBeamState`

来源：`include/1q/electronic_surveillance_radar/session/EsrSceneTypes.h`

| 字段 | 说明 | 影响功能 |
| --- | --- | --- |
| `center_az_deg` | 发射波束中心方位。 | 影响接收端是否处于发射波束覆盖方向。 |
| `center_el_deg` | 发射波束中心俯仰。 | 影响俯仰方向覆盖判定。 |
| `az_beamwidth_deg` | 方位波束宽度。 | 影响发射波束方位覆盖范围；必须 `>0`。 |
| `el_beamwidth_deg` | 俯仰波束宽度。 | 影响发射波束俯仰覆盖范围；必须 `>0`。 |
| `beam_state_valid` | 波束参数是否显式配置。 | 标记是否使用波束覆盖信息。 |

### 4.3 辐射源类型：`EsrSceneEmitter`

ESR 的 `scene` 不是目标点迹，而是可被电子侦察系统截获的辐射源事实。

| 字段 | 说明 | 影响功能 |
| --- | --- | --- |
| `emitter_id` | 辐射源标识。 | 用于输出关联、识别和调试定位；空字符串会被校验为 error。 |
| `pose` | 辐射源位置、速度与姿态。 | 影响传播距离、接收几何、波束覆盖和运动状态。 |
| `carrier_hz` | 发射中心频率，单位 Hz。 | 影响是否落入接收机频段、截获频率和信号分类；必须 `>0`。 |
| `bandwidth_hz` | 发射带宽，单位 Hz。 | 影响频域覆盖、检测带宽和信号特征；必须 `>0`。 |
| `tx_power_w` | 发射功率，单位 W。 | 影响接收功率、探测门限和动态范围；必须 `>0`。 |
| `pulse_width_s` | 脉宽，单位 s。 | 影响脉冲特征、截获门限和 PRI 一致性校验；必须 `>0`。 |
| `pri_s` | 脉冲重复间隔，单位 s。 | 影响脉冲时序识别；必须 `>0` 且不小于 `pulse_width_s`。 |
| `beam_state` | 当前发射波束状态。 | 影响方向覆盖和发射波束有效性。 |
| `is_emitting` | 当前周期是否发射。 | 为 `false` 时表示该辐射源本周期静默。 |

### 4.4 外部输入适配：`EsrExternalInputAdapter`

来源：`include/1q/electronic_surveillance_radar/session/EsrExternalInputAdapter.h`

| 类型/接口 | 字段或行为 | 说明 |
| --- | --- | --- |
| `EsrExternalPoseInput` | `platform_position_ecef_m` | 平台 ECEF 位置。构建器会转 LLA 作为局部 ENU 原点。 |
| `EsrExternalPoseInput` | `platform_velocity_mps` | 平台 ECEF 速度。转换为 ESR 局部速度。 |
| `EsrExternalPoseInput` | `platform_attitude_deg` | 平台姿态，Body->ENU。作为 ESR 局部参考姿态。 |
| `EsrExternalEmitterInput` | `emitter_position_ecef_m` | 辐射源 ECEF 位置。转换为 ESR 局部 `pose.position_m`。 |
| `EsrExternalEmitterInput` | `emitter_velocity_mps` | 辐射源 ECEF 速度。转换为 ESR 局部 `pose.velocity_mps`。 |
| `EsrExternalEmitterInput` | `emitter_attitude_deg` | 辐射源姿态。写入 `pose.attitude_deg`。 |
| `EsrExternalEmitterInput` | 发射参数与波束状态 | `carrier_hz`、`bandwidth_hz`、`tx_power_w`、`pulse_width_s`、`pri_s`、`beam_state`、`is_emitting` 原样写入场景辐射源。 |
| `TryMakeEsrPoseFromExternalKinematics(...)` | 平台转换。 | 输出 ESR 局部平台位姿。 |
| `TryMakeEsrSceneEmitterFromExternalInput(...)` | 辐射源转换。 | 输出 `EsrSceneEmitter`。 |

适配失败状态：

- `kNullOutput`：输出指针为空。
- `kCoordinateTransformFail`：坐标或速度转换失败。

### 4.5 一步构建器：`EsrCycleInputBuilder`

来源：`include/1q/electronic_surveillance_radar/session/EsrCycleInputBuilder.h`

注意事项：

- 构建器会根据平台 ECEF 位置生成局部参考原点。
- 构建器会把 `cycle_index` 置为 `0`，调用方可在构建后覆盖。
- ESR 构建器保留外部输入中的 `emitter_id`，不会改成数组索引。
- 有环境重载版本可同时写入完整 `EsrEnvironmentInput`。

### 4.6 ESR 输入校验

来源：`include/1q/electronic_surveillance_radar/session/EsrInputValidation.h`

重点校验项：

- `dt_sec` 必须有限且 `>0`。
- 平台位姿数值必须有限。
- `emitter_id` 不能为空。
- `carrier_hz`、`bandwidth_hz`、`tx_power_w`、`pulse_width_s`、`pri_s` 必须 `>0`。
- `pri_s` 不能小于 `pulse_width_s`。
- 波束宽度必须 `>0`。
- 辐射源数值字段必须有限。
- 环境观测字段必须合法。

## 5. 三模块输入差异对照

| 模块 | `scene` 元素 | 推荐直接输入形式 | 外部适配输入 | 主要影响链路 |
| --- | --- | --- | --- | --- |
| AR | `RadarSceneTarget` | 雷达局部位置/速度/RCS/斜距 | 平台 ECEF + 目标 ECEF 运动学 | 雷达方程、检测、关联、跟踪、航迹输出 |
| EOS | `EosSceneTarget` | 斜距 + 方位/俯仰 + 外观辐射 | 平台 ECEF + 目标 ECEF/LLA 位置 | 视场扫描、红外/可见光能量、杂散光、检测输出 |
| ESR | `EsrSceneEmitter` | 辐射源位姿 + 发射参数 + 波束状态 | 平台 ECEF + 辐射源 ECEF 运动学 | 频段截获、统计检测、传播/杂波/干扰、威胁评估 |

## 6. 使用建议

- 如果调用方已经有模块局部坐标和目标语义，直接填 `*CycleInput.scene` 最清晰。
- 如果调用方只有 ECEF/LLA 外部世界输入，优先使用 `*ExternalInputAdapter` 或 `*CycleInputBuilder`。
- 构建器默认 `cycle_index=0`，真实仿真周期应由调用方覆盖。
- AR/EOS 当前构建器使用列表索引作为目标 ID；需要稳定外部 ID 时应使用两步 adapter 或构建后覆盖。
- 目标输入和模型配置是两条不同路径：`*SessionConfig` 描述初始化模型，`*CycleInput` 描述每个周期来自外部世界的事实输入。
