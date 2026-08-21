# 1q Public Headers Guide

本目录定义 `1q` 静态库给外部工程使用的稳定头文件入口。
目标是让调用方快速判断"该去哪个文件夹做什么"。

## Top-level Rules

- 外部工程只应包含 `include/1q` 下头文件。
- `src/` 下头文件均视为内部实现，不提供兼容性承诺。
- 推荐优先从以下入口开始：
  - 基础契约：`1q/foundation/*`、`1q/coordinate/*`
  - 大气与传播：`1q/environment/*`
  - 追踪与回放：`1q/trace/*`、`1q/replay/*`
  - 机载雷达：`1q/airborne_radar/airborne_radar.hpp`
  - 电子侦察雷达：`1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp`
  - 光电传感器：`1q/electro_optical_sensor/electro_optical_sensor.hpp`
  - 合成孔径雷达：`1q/sar/sar.hpp`
  - 天基红外传感器：`1q/sbirs_sensor/sbirs_sensor.hpp`
  - 远程识别雷达：`1q/remote_identification_radar/remote_identification_radar.hpp`
  - 电子对抗：`1q/electronic_countermeasure/EcmSession.h`
  - 路径规划：`1q/navigation/navigation.hpp`
  - 多源融合：`1q/fusion/fusion.hpp`
  - 威胁评估：`1q/threat_assessment/threat_assessment.hpp`
  - 飞行动力学：`1q/flight_dynamic/FlightManager.h`

## Common

跨模块复用的基础设施目录（非传感器域）：

- `foundation/`：跨模块复用的基础类型与契约。
  - 位姿原语（`pose_types.h`）、扫描调度（`scan_schedule_types.h`）、
    输入校验基础类型（`validation_types.h`）、跨域会话形状契约（`SensorContract.h`）。
  轻量 JSON 解析器 `json_reader` 属于 example 共享便利层
  （`examples/common/json_reader.h`），不属于库 public surface。
- `coordinate/`：WGS-84 坐标系下位置/速度/姿态的帧间转换（LLA/ECEF/ENU/NED/NUE 互转）。
- `environment/`：大气模型抽象接口与传播物理算法（`IAtmosphereProvider`、
  `AtmosphericState`、`PropagationPhysics`、JSBSim 适配器）。作为大气类型的唯一公开来源。
- `trace/`：通用结构化记录 sink 接口（`TraceSink`）。
- `replay/`：回放 trace 写入器（`ReplayTrace.h`），用于可复现仿真。
- `flight_dynamic/`：JSBSim 动力学仿真入口（`FlightManager.h`），含 `config/`、
  `guidance/`、`model/`、`autopilot/` 子目录。
- `navigation/`：区域覆盖路径规划算法面（`AreaCoveragePlanner.h`），独立中立
  算法面，不绑定 flight_dynamic，输出度制中性航点（`RoutePoint`）。
- `fusion/`：多源关联 + 置信度融合算法面（`FusionEngine.h`），泛型探测记录
  （`DetectionRecord`），不感知传感器类型。
- `threat_assessment/`：目标威胁评估算法面（`ThreatEvaluator.h`），归一化加权和
  MADM，泛型属性帧（`ThreatEvaluationInput`），纯函数式无跨周期状态。

## Sensor Modules

各传感器域采用统一的 `config/` + `session/` 两域布局：

- `<module>/config/`：初始化与运行期配置入口（条件五域或四域 Config、RuntimePatch、
  ProfileConstants）；有静态安装指向几何的模块（SBIRS/AR/ESR）含 `orientation` 域，
  EOS/RIR/SAR 保持四域（见 `docs/common/contract.md`）。
  聚合头 `<module>_config.hpp`。
  会话配置直接赋值；运行期补丁直接写 `*RuntimeConfigPatch` 并显式设对应 `has_*`。
- `<module>/session/`：会话门面、周期输入/结果、输入校验、外部适配器、
  场景/输出类型、Trace/Replay 会话。聚合头 `<module>.hpp`。
- 各模块顶层 `<module>.hpp` 聚合稳定会话与配置 API；
  trace/replay 工具头（`*TraceSession.h`、`*ReplaySession.h`、`*DebugView.h`、
  `*LifecycleRecorder.h`）按需单独包含，不进入聚合头。

### Airborne Radar (AR)

- `airborne_radar/config/`：含 `ArProfileConstants`（Mission/Sensitivity 等
  语义档位）、`JammingSemantics`、`RadarOrientationConfig` 等雷达专用配置。
- `airborne_radar/session/`：会话门面、周期 IO、环境输入、外部适配器、
  决策 observation/response DTO、航迹生命周期记录。

### Electronic Surveillance Radar (ESR)

- `electronic_surveillance_radar/config/`：含 `EsrProfileConstants`
  （Mission/Sensitivity 语义档位）。
- `electronic_surveillance_radar/session/`：会话门面、周期 IO、环境输入、
  外部适配器、辐射源假设/观测类型（`EmitterHypothesis`、`EmitterObservation`）、
  输出边界类型。

### Electro Optical Sensor (EOS)

- `electro_optical_sensor/config/`：含 `EosProfileConstants`
  （Mission/Hardware 语义档位）。
- `electro_optical_sensor/session/`：会话门面、周期 IO、环境输入、
  外部适配器、场景/输出类型、检测生命周期记录。

### Synthetic Aperture Radar (SAR)

- `sar/config/`：含 `SarProfileConstants`（Mission/Processing 语义档位）。
- `sar/session/`：会话门面、周期输入/结果、输入校验、产品调试视图与生命周期记录。
  SAR 为批处理成像模型，一个周期对应一次完整合成孔径成像。
  @note SAR 的 `SarPlatformState`/`SarPointTarget` 使用 LLA，调用方填 LLA 即可；
  外部 IQ 的 `pulse_states` 要求 scene-center-relative ENU，由调用方直接填充，库内无
  ECEF/LLA 到 ENU 适配器。

### Space-Based Infrared Sensor (SBIRS)

- `sbirs_sensor/config/`：条件五域 SessionConfig（含静态 `orientation`）+ RuntimeConfigPatch
  （直接赋值 + 显式 `has_*`；orientation 不进 patch）。
- `sbirs_sensor/session/`：会话门面、周期 IO、外部适配器、场景/输出类型、
  检测生命周期记录、排除原因差分记录。

### Remote Identification Radar (RIR)

- `remote_identification_radar/config/`：四域配置（无 orientation 域）、RuntimePatch、
  SessionConfig 与 Profile 常量（直接赋值）。
- `remote_identification_radar/session/`：会话门面、周期 IO、输入校验、
  场景/输出/识别结果类型。

## Recommended Include Strategy

- 业务代码优先依赖：
  - `*/<module>.hpp`（模块统一入口，优先）
  - `*/config/<module>_config.hpp`（模块配置统一入口，优先）
  - `*/session/`（会话生命周期与输入校验）
- 避免无目的直接包含细粒度基础头，除非确实需要对应能力。
- trace/replay 能力按需单独包含对应工具头。

## Minimal Integration Checklist

- 机载雷达最小接入：
  - `#include "1q/airborne_radar/airborne_radar.hpp"`
  - `#include "1q/airborne_radar/config/airborne_radar_config.hpp"`
- 电子侦察雷达最小接入：
  - `#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"`
  - `#include "1q/electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"`
- 光电传感器最小接入：
  - `#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"`
  - `#include "1q/electro_optical_sensor/config/electro_optical_sensor_config.hpp"`
- 合成孔径雷达最小接入：
  - `#include "1q/sar/sar.hpp"`
  - `#include "1q/sar/config/sar_config.hpp"`
- 天基红外传感器最小接入：
  - `#include "1q/sbirs_sensor/sbirs_sensor.hpp"`
  - `#include "1q/sbirs_sensor/config/sbirs_sensor_config.hpp"`
- 远程识别雷达最小接入：
  - `#include "1q/remote_identification_radar/remote_identification_radar.hpp"`
  - `#include "1q/remote_identification_radar/config/remote_identification_radar_config.hpp"`
