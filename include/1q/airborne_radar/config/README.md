# Airborne Radar Config （Ar* 命名）

> 本目录所有公开配置类型使用 `Ar*` 命名。

## 公开主路径

调用方应仅通过以下头文件完成配置：

```text
config/
|-- ArHardwareConfig.h                 硬件固有能力（探测链路参数）
|-- ArMissionConfig.h                  任务态与波束运行态
|-- ArPolicyConfig.h                   调度/关联/跟踪/生命周期策略
|-- ArEnvironmentConfig.h              环境默认参数
|-- ArSessionConfig.h                  会话初始化配置壳（四域聚合）
|-- ArRuntimeConfigPatch.h             运行期可变参数补丁
|-- ArRuntimeConfigBuilder.h           运行期补丁 Builder
|-- ArSessionConfigBuilder.h           语义 Builder（Profile 输入）
|-- airborne_radar_config.hpp          统一入口头（聚合以上全部）
```

公开可见配置类型以 `airborne_radar::config` 为稳定命名空间，不再以 `expert` 子命名空间作为主路径。

## 核心类型

### 四域公开配置

| 域 | 头文件 | 说明 |
| --- | --- | --- |
| `hardware` | `ArHardwareConfig.h` | 硬件固有能力（探测链路参数） |
| `mission` | `ArMissionConfig.h` | 任务态与波束运行态（工作模式、指向与扫描） |
| `policy` | `ArPolicyConfig.h` | 调度/关联/跟踪/生命周期策略 |
| `environment` | `ArEnvironmentConfig.h` | 环境默认参数 |

### ArSessionConfig

[`ArSessionConfig.h`](ArSessionConfig.h)

会话初始化配置壳（四域聚合）：

- `hardware`：`ArHardwareConfig`
- `mission`：`ArMissionConfig`
- `policy`：`ArPolicyConfig`
- `environment`：`ArEnvironmentConfig`

## Builder 选择

### 语义 Builder

[`ArSessionConfigBuilder.h`](ArSessionConfigBuilder.h)

- 输入：`profiles::...Profile` 枚举
- 输出：`ArSessionConfig`（落到 `hardware/mission/policy/environment`）

适用：业务/任务层快速配置。

## Runtime Patch

[`ArRuntimeConfigPatch.h`](ArRuntimeConfigPatch.h) / [`ArRuntimeConfigBuilder.h`](ArRuntimeConfigBuilder.h)

支持运行期在不重建 `ArSession` 的前提下热更新参数：

- 整域覆盖：`mission`、`policy`、`environment_runtime_config`
- 叶子覆盖：工作模式、扫描中心、驻留中心、指令态波束宽度

规则：

- 同时存在整域和叶子时，先整域后叶子，叶子具有最终优先级
- 运行期补丁不等价于重建 `ArSession`

## 使用建议

- 业务/任务层优先：`ArSessionConfigBuilder`
- 常见场景推荐配置应在调用方业务层以具名函数封装，并返回 `ArSessionConfig` 传入 `ArSession::Create`
- 细粒度建模优先：直接字段赋值

## 推荐入口

- [`airborne_radar_config.hpp`](airborne_radar_config.hpp)
- [`ArSessionConfig.h`](ArSessionConfig.h)
- [`ArSessionConfigBuilder.h`](ArSessionConfigBuilder.h)
