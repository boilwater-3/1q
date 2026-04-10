# 1q Public Headers Guide

本目录定义 `1q` 静态库给外部工程使用的稳定头文件入口。  
目标是让调用方快速判断“该去哪个文件夹做什么”。

## Top-level Rules

- 外部工程只应包含 `include/1q` 下头文件。
- `src/` 下头文件均视为内部实现，不提供兼容性承诺。
- 推荐优先从以下入口开始：
  - 通用能力：`1q/common/*`
  - 机载雷达：`1q/airborne_radar/airborne_radar.hpp`
  - 电子侦察雷达：`1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp`
  - 光电传感器：`1q/electro_optical_sensor/electro_optical_sensor.hpp`

## Common

- `common/`：跨模块复用的基础类型与坐标转换。
- `common/trace/`：跨模块复用的追踪输出接口（TraceSink）。

## Airborne Radar

- `airborne_radar/config/`：初始化与运行期配置入口（Builder、预设、扫描/姿态配置）。
- `airborne_radar/session/`：会话门面与周期结果。
- `airborne_radar/core/context/`：单周期输入与输入校验。
- `airborne_radar/core/controller/`：控制器门面与输出读取接口。
- `airborne_radar/environment/`：环境输入与构造辅助。
- `airborne_radar/decision/pipeline/`：决策管线契约类型。
- `airborne_radar/signal/pipeline/`：信号管线契约类型。
- `airborne_radar/common/`：目标、输出等公共域模型（纯输入输出契约为主）。
- `airborne_radar/extension/`：外部接管行为接口相关类型（如 control 指令/控制真值）。
- `airborne_radar/tools/`：调试/追踪辅助封装。

## Electronic Surveillance Radar (ESR)

- `electronic_surveillance_radar/config/`：配置与 Builder 入口（推荐从这里拿配置类型与 Builder）。
- `electronic_surveillance_radar/core/session/`：会话门面与结果对象。
- `electronic_surveillance_radar/core/context/`：周期输入与校验。
- `electronic_surveillance_radar/core/controller/`：控制门面。
- `electronic_surveillance_radar/environment/`：环境输入契约。
- `electronic_surveillance_radar/pipeline/`：拦截管线契约。
- `electronic_surveillance_radar/common/`：观测、假设、姿态等公共域模型。
- `electronic_surveillance_radar/tools/`：调试/追踪辅助封装。

## Electro Optical Sensor (EOS)

- `electro_optical_sensor/config/`：配置与 Builder 入口（推荐从这里拿配置类型与 Builder）。
- `electro_optical_sensor/session/`：会话门面与 trace 包装器。
- `electro_optical_sensor/model/`：周期输入、输入校验、周期结果等核心数据模型。
- `electro_optical_sensor/extension/`：控制器、pipeline、environment service 等扩展接口契约。
- `electro_optical_sensor/environment/`：环境契约类型（不含扩展接口）。
- `electro_optical_sensor/foundation/`：辐射传输、噪声、光学等基础模型。
- `electro_optical_sensor/output/`：输出帧模型。
- `electro_optical_sensor/utils/`：坐标工具等通用工具能力。

## Recommended Include Strategy

- 业务代码优先依赖：
  - `*/<module>.hpp`（模块统一入口，优先）
  - `*/config/*_config.hpp`（模块配置统一入口，优先）
  - `*/session/`（会话生命周期与输入校验）
  - `*/extension/`（可替换扩展 seam）
- 避免无目的直接包含 `common/utils` 等细粒度头，除非确实需要对应工具能力。

## Minimal Integration Checklist

- 机载雷达最小接入：
  - `#include "1q/airborne_radar/airborne_radar.hpp"`
  - `#include "1q/airborne_radar/config/airborne_radar_config.hpp"`
- ESR 最小接入：
  - `#include "1q/electronic_surveillance_radar/electronic_surveillance_radar.hpp"`
  - `#include "1q/electronic_surveillance_radar/config/electronic_surveillance_radar_config.hpp"`
- EOS 最小接入：
  - `#include "1q/electro_optical_sensor/electro_optical_sensor.hpp"`
  - `#include "1q/electro_optical_sensor/config/electro_optical_sensor_config.hpp"`
