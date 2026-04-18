# Electro Optical Sensor 架构改造执行计划

## 背景与目标

`electro_optical_sensor` 当前对外配置已经较接近目标架构，但公开结构仍分散为 `optical / scan / pointing / detection / stray_light / environment` 六块，外部模型尚未收敛为统一的四域语言。结合当前统一架构理念，本模块目标是将公开配置收敛为：

- `hardware`
- `mission`
- `policy`
- `environment`

同时补足两项当前不足：

- 面向高级调用方的细粒度详细配置入口
- 更清晰的 runtime patch 边界与统一覆盖规则

本模块改造同样按“不保兼容”执行，不保留长期双轨接口。

## 当前结构摘要

当前公开配置入口主要集中在以下类型和文件：

- `include/1q/electro_optical_sensor/config/EosSessionConfig.h`
- `include/1q/electro_optical_sensor/config/EosSessionConfigBuilder.h`
- `include/1q/electro_optical_sensor/config/EosRuntimeConfigPatch.h`
- `include/1q/electro_optical_sensor/config/EosRuntimeConfigBuilder.h`
- `include/1q/electro_optical_sensor/config/EosOpticalConfig.h`
- `include/1q/electro_optical_sensor/config/EosScanPolicyConfig.h`
- `include/1q/electro_optical_sensor/config/EosPointingConfig.h`
- `include/1q/electro_optical_sensor/config/EosDetectionPolicyConfig.h`
- `include/1q/electro_optical_sensor/config/EosStrayLightPolicyConfig.h`
- `include/1q/electro_optical_sensor/config/EosEnvironmentPolicyConfig.h`

现状分层如下：

- `session::EosSessionConfig`
  - `optical`
  - `scan`
  - `pointing`
  - `detection`
  - `stray_light`
  - `environment`
- `config::EosSessionConfigBuilder`
  - 只提供高层组装入口
- `session::EosRuntimeConfigPatch`
  - 只提供若干叶子热更新字段
- `src/electro_optical_sensor/runtime/EosPipelineConfigMapper.cpp`
  - 将散开的 session config 逐项映射到 `extension::EosPipelineConfig`
- `src/electro_optical_sensor/runtime/EosRuntimeConfigResolver.cpp`
  - 将散开的 runtime patch 逐项叠加到 `EosSessionConfig`

## 目标公开 API 草案

目标结构收敛为：

```cpp
namespace electro_optical_sensor {
namespace config {

struct EosHardwareConfig;
struct EosMissionConfig;
struct EosPolicyConfig;
struct EosEnvironmentConfig;

}  // namespace config

namespace session {

struct EosSessionConfig {
  config::EosHardwareConfig hardware{};
  config::EosMissionConfig mission{};
  config::EosPolicyConfig policy{};
  config::EosEnvironmentConfig environment{};
};

}  // namespace session
}  // namespace electro_optical_sensor
```

Builder 收敛为两类：

- `EosSessionConfigBuilder`
  - 作为语义入口保留
  - 输入 work mode、profile、preset 等高层语义
- `EosDetailedSessionConfigBuilder`
  - 新增细粒度入口
  - 直接编辑 `hardware / mission / policy / environment`
  - 覆盖高级调用方对具体硬件规格和详细策略参数的配置需求

运行时配置收敛为：

- `EosRuntimeConfigPatch`
  - 支持整块覆盖 `mission / policy / environment`
  - 支持高频叶子字段覆盖：`work_mode`、`scan_rate_deg_per_sec`、`frame_rate_hz`
  - 统一优先级：先整块，后叶子

## 现状到目标的映射表

### 公开会话配置映射

| 当前类型 | 目标域 | 迁移原则 |
| --- | --- | --- |
| `EosOpticalHardwareConfig` | `EosHardwareConfig` | 当前 optical 能力整体迁入 hardware |
| `EosScanPolicyConfig` | `EosMissionConfig` | 工作模式、FOV、扫描率、帧率归入 mission |
| `EosPointingConfig` | `EosMissionConfig` | 扫描起止角、中心俯仰、视轴俯角归入 mission |
| `EosDetectionPolicyConfig` | `EosPolicyConfig::detection` | 保留 profile，并补充详细检测参数 |
| `EosStrayLightPolicyConfig` | `EosPolicyConfig::stray_light` | 保留 profile，并补充详细抑制参数 |
| `EosEnvironmentPolicyConfig` | `EosEnvironmentConfig` | 环境模型类型、预设及扩展标志整体进入 environment |

### 运行时配置映射

| 当前字段 | 目标位置 | 迁移原则 |
| --- | --- | --- |
| `work_mode` | `mission.work_mode` | 保留热更新能力 |
| `scan_rate_deg_per_sec` | `mission.scan_rate_deg_per_sec` | 保留 |
| `frame_rate_hz` | `mission.frame_rate_hz` | 保留 |
| `detection_profile` | `policy.detection.profile` | 保留 |
| `stray_light_profile` | `policy.stray_light.profile` | 保留 |
| `environment_model_type` | `environment.model_type` | 保留 |
| `environment_preset` | `environment.preset` | 保留 |

## 分阶段实施步骤

### 阶段 1：重塑公开配置类型

- 新增 `EosHardwareConfig`、`EosMissionConfig`、`EosPolicyConfig`、`EosEnvironmentConfig`。
- 将当前 `EosSessionConfig` 改为四域结构。
- `EosPolicyConfig` 至少拆分为：
  - `detection`
  - `stray_light`
- 在新类型中补足详细参数字段，避免 detailed builder 仍依赖旧散装结构。

完成判据：

- 外部可以只通过新 `EosSessionConfig` 表达当前全部公开能力。
- 四域分层与目标统一架构保持一致。

### 阶段 2：建立双 Builder 体系

- 保留并重写 `EosSessionConfigBuilder`，继续承担语义式构造。
- 新增 `EosDetailedSessionConfigBuilder`，支持直接填写详细硬件参数和详细策略参数。
- 所有 builder 示例和注释统一使用 `hardware / mission / policy / environment` 术语。

完成判据：

- 低心智负担用户仍可使用 profile/preset 构造。
- 高级用户可直接配置具体光学硬件参数、检测阈值和杂散光抑制细节。

### 阶段 3：重接内部映射责任

- 重写 `EosPipelineConfigMapper.cpp`，使其从新 `EosSessionConfig` 四域结构读取数据。
- 不再继续依赖“把旧 session config 拆成六块再拼装”的路径。
- 重写 `EosRuntimeConfigResolver.cpp`，使其围绕新的 `EosRuntimeConfigPatch` 工作。
- 明确 patch 规则：
  - 先整块覆盖 `mission / policy / environment`
  - 再应用叶子快捷字段
  - 数值校验失败时整体拒绝 patch

完成判据：

- `EosSessionFactory::Create(...)` 能从新配置壳正确生成 `extension::EosPipelineConfig`。
- runtime patch 路径不再依赖旧结构命名。

### 阶段 4：更新消费者与回归测试

- 更新 `tests/consumer/eos_session_consumer.cpp`，改用新 `EosSessionConfig` 和新 builder。
- 更新 `tests/contract/public_headers_smoke_test.cpp` 中 EOS 配置相关断言。
- 回归 session 创建、Step、StepWithResult、runtime patch 等核心路径。

完成判据：

- consumer 场景完全使用新公开语言。
- 头文件冒烟测试覆盖新的公开入口。

### 阶段 5：清理旧公开结构

- 删除旧的散装公开配置概念作为主入口的说明。
- 删除所有只服务于旧公开结构的桥接逻辑。
- 如果旧 `optical / scan / pointing / detection / stray_light / environment` 头仍保留，需明确其角色不是主入口，随后在同一改造周期清理。

完成判据：

- EOS 公开配置语言和 AR/ESR 目标结构同构。
- 仓库中不再存在长期双轨对外接口。

## 测试与验收标准

### 配置类型层

- 新 `EosSessionConfig` 完整覆盖旧 `optical / scan / pointing / detection / stray_light / environment` 能力。
- 新 `EosDetailedSessionConfigBuilder` 能表达 profile 之外的详细硬件和详细策略参数。
- 新 `EosRuntimeConfigPatch` 只暴露允许运行期修改的域和字段。

### 行为层

- session 初始化路径能正确将新四域配置映射到 `extension::EosPipelineConfig`。
- runtime patch 的覆盖优先级和数值校验规则统一且可测试。
- 原 consumer 示例中的模式切换、扫描率修改、环境切换、杂散光切换场景都能用新 API 重写并通过。

### 需要更新和回归的测试

- `tests/consumer/eos_session_consumer.cpp`
- `tests/contract/public_headers_smoke_test.cpp`
- 任何直接依赖旧 `EosSessionConfig` 结构的 session/runtime 测试

### 验收标准

- 选定 preset 下构建与相关测试通过。
- EOS 公开配置入口与 AR/ESR 目标架构同构。
- 新文档和示例统一以四域模型描述 EOS。

## 风险、边界与默认决策

- `hardware` 域承载固有光学与探测器规格，不承载运行期扫描控制。
- `mission` 域承载工作模式、扫描控制、指向控制和高频热更新字段。
- `policy` 域承载检测与杂散光策略，包括 profile 和详细门限/参数。
- `environment` 域承载环境模型类型、预设和环境扩展相关参数。
- 不保留长期兼容层；旧结构一旦退出主入口，调用方需同步迁移。
