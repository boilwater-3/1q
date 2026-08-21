# Airborne Radar Config （Ar* 命名）

> 本目录所有公开配置类型使用 `Ar*` 命名。

## 公开主路径

调用方应仅通过以下头文件完成配置：

```text
config/
|-- ArHardwareConfig.h                 硬件固有能力（发射机/天线/接收机/RCS；门限在 policy.detection）
|-- ArMissionConfig.h                  任务态与波束运行态（work_mode / scan_center / 指令波束宽）
|-- ArOrientationConfig.h              静态安装指向（安装角 / 限位 / 稳定 / 扫描程序）
|-- ArPolicyConfig.h                   调度/关联/跟踪/生命周期策略
|-- ArEnvironmentConfig.h              环境默认参数
|-- ArSessionConfig.h                  会话初始化配置壳（条件五域聚合）
|-- ArRuntimeConfigPatch.h             运行期可变参数补丁（显式 has_*；不含 orientation）
|-- ArProfileConstants.h               语义档位常量（直接赋值）
|-- airborne_radar_config.hpp          统一入口头（聚合以上全部）
```

公开可见配置类型以 `airborne_radar::config` 为稳定命名空间，不再以 `expert` 子命名空间作为主路径。

## 核心类型

### 条件五域公开配置

| 域 | 头文件 | 说明 | 内部映射去向 |
| --- | --- | --- | --- |
| `hardware` | `ArHardwareConfig.h` | 硬件固有能力（发射机、天线、接收机、RCS 物理）。**检测门限在 policy.detection** | `InternalExecutionConfig::detection.engineering`（与 policy.detection 合并） |
| `mission` | `ArMissionConfig.h` | 运行期任务态（工作模式、基准指向、指令波束宽） | `InternalExecutionConfig::sensor_enabled` + `detection.orientation`（拼装态字段） |
| `orientation` | `ArOrientationConfig.h` | 静态安装指向与稳定（安装角、机/电限位、扫描起始/顺序、稳定方式） | `detection.orientation`（拼装态静态字段）；**不进 RuntimeConfigPatch** |
| `policy` | `ArPolicyConfig.h` | 调度/关联/跟踪/生命周期策略 | `detection.beam_control`、`association`、`tracking`、`lifecycle`、`decision_control` |
| `environment` | `ArEnvironmentConfig.h` | 环境默认参数 | **独立路径**：`EnvironmentService`（不经过 `MapSessionToExecution`） |

内部 pipeline 消费的是拼装后的 `ArEffectiveOrientationConfig`（静态 orientation + mission 运行期指向字段），不是 SessionConfig 顶层域。

### ArSessionConfig

[`ArSessionConfig.h`](ArSessionConfig.h)

会话初始化配置壳（条件五域聚合）：

- `hardware`：`ArHardwareConfig`
- `mission`：`ArMissionConfig`
- `orientation`：`ArOrientationConfig`（静态；不进 runtime patch）
- `policy`：`ArPolicyConfig`
- `environment`：`ArEnvironmentConfig`
- 顶层 `sensor_enabled`：电源状态（COMMON-OQ-4）

## 会话配置

直接构造/赋值 [`ArSessionConfig.h`](ArSessionConfig.h)；语义档位见
[`ArProfileConstants.h`](ArProfileConstants.h)（整域赋给 `hardware`/`mission`/`policy` 等）。

## Runtime Patch

[`ArRuntimeConfigPatch.h`](ArRuntimeConfigPatch.h)

支持运行期在不重建 `ArSession` 的前提下热更新参数：

- 整域覆盖：`mission`、`policy`、`environment_runtime_config`（须设对应 `has_*`）
- 叶子覆盖：工作模式、扫描中心、驻留中心、指令态波束宽度（须设对应 `has_*`）
- **不含** `orientation`（静态安装指向域，会话期固定）

规则：

- 同时存在整域和叶子时，先整域后叶子，叶子具有最终优先级
- 运行期补丁不等价于重建 `ArSession`

## 使用建议

- 业务/任务层：直接赋值 `ArSessionConfig` + `ArProfileConstants`
- 常见场景推荐配置应在调用方业务层以具名函数封装，并返回 `ArSessionConfig` 传入 `ArSession::Create`
- 运行期：直接写 `ArRuntimeConfigPatch` 并显式设 `has_*`

## 推荐入口

- [`airborne_radar_config.hpp`](airborne_radar_config.hpp)
- [`ArSessionConfig.h`](ArSessionConfig.h)
- [`ArProfileConstants.h`](ArProfileConstants.h)
- [`ArRuntimeConfigPatch.h`](ArRuntimeConfigPatch.h)
