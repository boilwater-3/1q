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
  - 飞行动力学：`1q/flight_dynamic/FlightManager.h`

## Common

跨模块复用的基础设施目录（非传感器域）：

- `foundation/`：跨模块复用的基础类型与契约。
  - 位姿原语（`pose_types.h`）、扫描调度（`scan_schedule_types.h`）、
    输入校验基础类型（`validation_types.h`）、轻量 JSON 值树（`json_reader.h`）、
    跨域会话形状契约（`SensorContract.h`）。
- `coordinate/`：WGS-84 坐标系下位置/速度/姿态的帧间转换（LLA/ECEF/ENU/NED/NUE 互转）。
- `environment/`：大气模型抽象接口与传播物理算法（`IAtmosphereProvider`、
  `AtmosphericState`、`PropagationPhysics`、JSBSim 适配器）。作为大气类型的唯一公开来源。
- `trace/`：通用结构化记录 sink 接口（`TraceSink`）。
- `replay/`：回放 trace 写入器（`ReplayTrace.h`），用于可复现仿真。
- `flight_dynamic/`：JSBSim 动力学仿真入口（`FlightManager.h`），含 `config/`、
  `guidance/`、`model/`、`autopilot/` 子目录。

## Sensor Modules

四个传感器域采用统一的 `config/` + `session/` 两域布局：

- `<module>/config/`：初始化与运行期配置入口（四域 Config、RuntimeBuilder、
  RuntimePatch、SessionConfig、SessionConfigBuilder）。聚合头 `<module>_config.hpp`。
- `<module>/session/`：会话门面、周期输入/结果、输入校验、外部适配器、
  场景/输出类型、Trace/Replay 会话。聚合头 `<module>.hpp`。
- 各模块顶层 `<module>.hpp` 聚合稳定会话与配置 API；
  trace/replay 工具头（`*TraceSession.h`、`*ReplaySession.h`、`*DebugView.h`、
  `*LifecycleRecorder.h`）按需单独包含，不进入聚合头。

### Airborne Radar (AR)

- `airborne_radar/config/`：含 `RadarSessionConfigBuilder`（Mission/Sensitivity 等
  语义档位）、`JammingSemantics`、`RadarOrientationConfig` 等雷达专用配置。
- `airborne_radar/session/`：会话门面、周期 IO、环境输入、外部适配器、
  决策 observation/response DTO、航迹生命周期记录。

### Electronic Surveillance Radar (ESR)

- `electronic_surveillance_radar/config/`：含 `EsrSessionConfigBuilder`
  （Mission/Sensitivity 语义档位）。
- `electronic_surveillance_radar/session/`：会话门面、周期 IO、环境输入、
  外部适配器、辐射源假设/观测类型（`EmitterHypothesis`、`EmitterObservation`）、
  输出边界类型。

### Electro Optical Sensor (EOS)

- `electro_optical_sensor/config/`：含 `EosSessionConfigBuilder`
  （Mission/Hardware 语义档位）。
- `electro_optical_sensor/session/`：会话门面、周期 IO、环境输入、
  外部适配器、场景/输出类型、检测生命周期记录。

### Synthetic Aperture Radar (SAR)

- `sar/config/`：含 `SarSessionConfigBuilder`（Mission/Processing 语义档位）。
- `sar/session/`：会话门面、周期输入/结果、输入校验、脉冲坐标适配器
  （`SarExternalInputAdapter` / `SarCycleInputAdapter`，把外部 ECEF/LLA 脉冲运动学
  转换为 scene-center-relative ENU 本地直角坐标）、产品调试视图与生命周期记录。
  SAR 为批处理成像模型，一个周期对应一次完整合成孔径成像。
  @note SAR 适配器只覆盖脉冲状态，不像 AR/ESR/EOS 那样适配平台/目标——因为
  SAR 的 `SarPlatformState`/`SarPointTarget` 内部直接存 LLA，调用方填 LLA 即可；
  唯独外部 IQ 的 `pulse_states` 要求 scene-center ENU，故适配器聚焦于此。

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
