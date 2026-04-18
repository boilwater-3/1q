# Electronic Surveillance Radar 架构改造执行计划

## 背景与目标

`electronic_surveillance_radar` 当前的公开结构已经具有较好的领域感，公开模型包含 `hardware / mission / detection / environment`。本次改造的重点不是推倒重来，而是在当前基础上完成两件事：

- 将 `detection` 域扩展并收敛为统一的 `policy` 域
- 建立与 AR/EOS 一致的双 builder 和 runtime patch 设计

目标公开语言最终统一为：

- `hardware`
- `mission`
- `policy`
- `environment`

本模块同样按“不保兼容”执行，不保留旧公开接口长期并存。

## 当前结构摘要

当前公开配置入口主要集中在以下类型和文件：

- `include/1q/electronic_surveillance_radar/config/EsrSessionConfig.h`
- `include/1q/electronic_surveillance_radar/config/EsrSessionConfigBuilder.h`
- `include/1q/electronic_surveillance_radar/config/EsrRuntimeConfigPatch.h`
- `include/1q/electronic_surveillance_radar/config/EsrRuntimeConfigBuilder.h`
- `include/1q/electronic_surveillance_radar/config/EsrHardwareConfig.h`
- `include/1q/electronic_surveillance_radar/config/EsrMissionControlConfig.h`
- `include/1q/electronic_surveillance_radar/config/EsrScanPolicyConfig.h`
- `include/1q/electronic_surveillance_radar/config/EsrDetectionPolicyConfig.h`
- `include/1q/electronic_surveillance_radar/config/EsrEnvironmentPolicyConfig.h`

现状分层如下：

- `session::EsrSessionConfig`
  - `hardware`
  - `mission`
  - `detection`
  - `environment`
- `config::EsrSessionConfigBuilder`
  - 主要提供高层组装入口
- `session::EsrRuntimeConfigPatch`
  - 主要由 mission 叶子字段和部分 environment 字段构成
- `src/electronic_surveillance_radar/session/EsrSessionConfigResolver.cpp`
  - 从 session config 解析 runtime config、pipeline config、environment config
- `src/electronic_surveillance_radar/session/EsrRuntimeConfigResolver.cpp`
  - 对 resolved config 进行 patch 叠加

## 目标公开 API 草案

目标结构收敛为：

```cpp
namespace electronic_surveillance_radar {
namespace config {

struct EsrHardwareConfig;
struct EsrMissionConfig;
struct EsrPolicyConfig;
struct EsrEnvironmentConfig;

}  // namespace config

namespace session {

struct EsrSessionConfig {
  config::EsrHardwareConfig hardware{};
  config::EsrMissionConfig mission{};
  config::EsrPolicyConfig policy{};
  config::EsrEnvironmentConfig environment{};
};

}  // namespace session
}  // namespace electronic_surveillance_radar
```

Builder 收敛为两类：

- `EsrSessionConfigBuilder`
  - 保留语义入口
  - 面向工作模式、检测档位、环境预设和型号级配置
- `EsrDetailedSessionConfigBuilder`
  - 新增详细配置入口
  - 面向硬件参数、扫描参数、详细检测参数和环境详细参数

运行时配置收敛为：

- `EsrRuntimeConfigPatch`
  - 支持整块覆盖 `mission / policy / environment`
  - 支持高频叶子覆盖：`sensor_enabled`、`work_mode`、`scan_rate_hz`、扫描中心、扫描边界
  - 统一规则：先整块，后叶子

## 现状到目标的映射表

### 公开会话配置映射

| 当前类型 | 目标域 | 迁移原则 |
| --- | --- | --- |
| `EsrHardwareConfig` | `EsrHardwareConfig` | 保留名称与主体内容 |
| `EsrMissionControlConfig` | `EsrMissionConfig` | `power_on/work_mode/scan` 收敛为 mission |
| `EsrScanPolicyConfig` | `EsrMissionConfig` | 作为 mission 子域或并入 mission 本体 |
| `EsrDetectionPolicyConfig` | `EsrPolicyConfig::detection` | detection 升格为统一 policy 域的一个子域 |
| `EsrEnvironmentPolicyConfig` | `EsrEnvironmentConfig` | 环境预设和环境详细参数整体进入 environment |

### 运行时配置映射

| 当前字段 | 目标位置 | 迁移原则 |
| --- | --- | --- |
| `sensor_enabled` | `mission.sensor_enabled` 或快捷叶子 | 保留 |
| `work_mode` | `mission.work_mode` 或快捷叶子 | 保留 |
| `scan_rate_hz` | `mission.scan_rate_hz` 或快捷叶子 | 保留 |
| `scan_start_position` | `mission.scan_start_position` 或快捷叶子 | 保留 |
| `scan_sequence` | `mission.scan_sequence` 或快捷叶子 | 保留 |
| `scan_center_az_deg/scan_center_el_deg` | `mission` | 保留 |
| `explicit scan bounds` | `mission` | 保留 |
| `preset` | `environment.preset` | 保留 |
| `atmospheric_physics/atmospheric_context` | `environment` | 保留并明确为环境详细参数 |

## 分阶段实施步骤

### 阶段 1：收敛公开配置模型

- 将 `EsrSessionConfig` 从 `hardware / mission / detection / environment` 收敛为 `hardware / mission / policy / environment`。
- 新增 `EsrPolicyConfig`，至少包含 `detection` 子域。
- 根据当前解析实现补足 policy 详细参数，避免检测能力仍只能靠 profile 表达。
- 明确 `EsrMissionConfig` 是否内含 scan 子结构；无论内部组织如何，对外都应表现为单一 mission 域。

完成判据：

- ESR 公开配置外形与 AR/EOS 目标架构一致。
- 外部仍能表达当前全部扫描、工作模式和环境配置能力。

### 阶段 2：建立双 Builder 体系

- 保留并重写 `EsrSessionConfigBuilder`，继续服务于语义入口。
- 新增 `EsrDetailedSessionConfigBuilder`。
- 详细 builder 需支持：
  - 详细硬件参数
  - 详细扫描与任务控制参数
  - 详细检测策略参数
  - 详细环境参数

完成判据：

- 调用方既可按 profile/preset 构造，也可直接给定详细参数。
- builder 示例和注释统一使用四域语言。

### 阶段 3：重构解析责任

- 重写 `EsrSessionConfigResolver.cpp`，围绕新的 `EsrSessionConfig` 解析：
  - `runtime_config`
  - `pipeline_config`
  - `environment_model_config`
- 不再延续当前“mission + detection + environment 分散映射”的责任切分方式。
- 重写 `EsrRuntimeConfigResolver.cpp`，使其围绕新的 `EsrRuntimeConfigPatch` 工作。
- 统一 patch 规则：
  - 先整块域覆盖
  - 再叶子覆盖
  - 非法显式扫描边界或非有限数值时整体拒绝

完成判据：

- session 解析责任按新分层清晰落地。
- runtime patch 行为可预测、可测试。

### 阶段 4：更新消费者与回归测试

- 更新 `tests/consumer/esr_session_consumer.cpp`。
- 更新 `tests/contract/public_headers_smoke_test.cpp` 中 ESR 配置路径。
- 补充或更新覆盖以下行为的测试：
  - 工作模式切换
  - 传感器开关
  - 扫描中心修改
  - 显式扫描边界设置与清除
  - 环境预设切换
  - 环境详细参数热更新

完成判据：

- 现有 consumer 场景可用新 API 全量重写。
- ESR 公开配置相关头文件冒烟测试通过。

### 阶段 5：清理旧公开结构

- 移除旧 `detection` 作为顶层域的对外描述。
- 删除只为兼容旧公开会话配置布局而存在的桥接代码。
- 将所有公开文档、示例和注释统一改为 `hardware / mission / policy / environment`。

完成判据：

- ESR 不再保留长期双轨公开接口。
- 与 AR/EOS 的公开配置分层和命名保持一致。

## 测试与验收标准

### 配置类型层

- 新 `EsrSessionConfig` 必须完整覆盖当前 `hardware / mission / detection / environment` 的全部公开能力。
- 新 `EsrDetailedSessionConfigBuilder` 必须能表达当前公开详细扫描和硬件参数能力，并补充详细检测策略能力。
- 新 `EsrRuntimeConfigPatch` 只暴露允许运行期修改的域和字段。

### 行为层

- session 初始化路径能从新的四域结构正确映射到 `InterceptPipelineConfig`、`InterceptRuntimeConfig` 与 `EsrEnvironmentModelConfig`。
- runtime patch 满足优先级和校验规则。
- 显式扫描边界、中心扫描和工作模式切换场景在新 API 下保持可验证行为。

### 需要更新和回归的测试

- `tests/consumer/esr_session_consumer.cpp`
- `tests/contract/public_headers_smoke_test.cpp`
- 任何直接依赖旧 `EsrSessionConfig` 顶层 `detection` 域的测试

### 验收标准

- 选定 preset 下构建与相关测试通过。
- ESR 公开配置分层与 AR/EOS 同构。
- 文档中的每个改造阶段都能对应到明确代码任务和完成判据。

## 风险、边界与默认决策

- `hardware` 域继续承载装备固有接收能力和天线几何能力。
- `mission` 域承载任务控制、扫描控制和运行时热更新控制量。
- `policy` 域承载检测策略及未来可扩展的其他策略子域。
- `environment` 域承载环境预设和环境详细参数。
- 不保留旧顶层 `detection` 作为长期公开结构；统一向 `policy` 收敛。
