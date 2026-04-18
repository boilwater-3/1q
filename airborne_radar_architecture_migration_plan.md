# Airborne Radar 架构改造执行计划

## 背景与目标

`airborne_radar` 当前对外公开的配置语言以 `pipeline_config / expert / orientation` 为核心，表达能力强，但对外合同明显带有内部流水线组织痕迹。结合当前统一架构理念，本模块的目标不是削弱细粒度配置能力，而是将公开语言收敛为领域语义稳定、与内部实现解耦的四域模型：

- `hardware`
- `mission`
- `policy`
- `environment`

本次改造按仓库现有重构政策执行，不保留长期双轨兼容，不设计临时适配层。目标是让外部仍可精确配置不同体制、不同型号、不同性能档位的机载雷达，但公开 API 不再以 `pipeline` 和 `expert` 作为主入口。

## 当前结构摘要

当前公开配置入口主要集中在以下类型和文件：

- `include/1q/airborne_radar/config/RadarSessionConfig.h`
- `include/1q/airborne_radar/config/PipelineConfig.h`
- `include/1q/airborne_radar/config/RadarSessionConfigBuilder.h`
- `include/1q/airborne_radar/config/RadarExpertSessionConfigBuilder.h`
- `include/1q/airborne_radar/config/RadarRuntimeConfigBuilder.h`
- `include/1q/airborne_radar/config/expert/ExpertPipelineConfig.h`
- `include/1q/airborne_radar/model/RadarOrientationConfig.h`
- `include/1q/airborne_radar/environment/EnvironmentConfig.h`

现状分层如下：

- `session::RadarSessionConfig`
  - `pipeline_config`
    - `expert::ExpertPipelineConfig`
    - `model::RadarOrientationConfig`
  - `environment::EnvironmentDefaultConfig`
- `config::RadarSessionConfigBuilder`
  - 语义档位输入，`Build()` 时翻译为 `expert`
- `config::RadarExpertSessionConfigBuilder`
  - 直接编辑 `expert` 和 `orientation`
- `config::RadarRuntimeConfigPatch`
  - 支持整块覆盖 `PipelineConfig`
  - 支持覆盖 `EnvironmentRuntimeConfigPatch`
  - 支持若干 orientation 叶子字段热更新

内部关键实现路径：

- `src/airborne_radar/session/RadarSessionConfigBuilder.cpp`
- `src/airborne_radar/session/RuntimeConfigResolver.cpp`
- `src/airborne_radar/session/RadarSessionCompositionRoot.cpp`

## 目标公开 API 草案

目标结构收敛为：

```cpp
namespace airborne_radar {
namespace config {

struct RadarHardwareConfig;
struct RadarMissionConfig;
struct RadarPolicyConfig;
using RadarEnvironmentConfig = environment::EnvironmentDefaultConfig;

}  // namespace config

namespace session {

struct RadarSessionConfig {
  config::RadarHardwareConfig hardware{};
  config::RadarMissionConfig mission{};
  config::RadarPolicyConfig policy{};
  config::RadarEnvironmentConfig environment{};
};

}  // namespace session
}  // namespace airborne_radar
```

Builder 收敛为两类：

- `RadarSessionConfigBuilder`
  - 保留语义入口
  - 输入 profile / preset / mission-level 语义
  - 输出新的 `session::RadarSessionConfig`
- `RadarDetailedSessionConfigBuilder`
  - 替代当前 `RadarExpertSessionConfigBuilder`
  - 直接编辑 `hardware / mission / policy / environment`
  - 不再以 `expert` 作为对外概念

运行时配置收敛为：

- `RadarRuntimeConfigPatch`
  - 支持整块覆盖 `mission / policy / environment`
  - 支持高频叶子覆盖：`work_mode`、`scan_center_deg`、`dwell_center_deg`、`commanded_beamwidth_deg`、`commanded_beamwidth_enabled`
  - 统一优先级：先整块，后叶子

## 现状到目标的映射表

### 公开会话配置映射

| 当前类型 | 目标域 | 迁移原则 |
| --- | --- | --- |
| `config::PipelineConfig::expert.detection` | `RadarHardwareConfig` + `RadarPolicyConfig::detection` | 装备固有能力放入 `hardware`，任务/检测策略阈值放入 `policy` |
| `config::PipelineConfig::expert.beam_control` | `RadarMissionConfig` + `RadarPolicyConfig::beam` | 波束调度策略放入 `policy`，当前任务态/扫描态放入 `mission` |
| `config::PipelineConfig::expert.tracking` | `RadarPolicyConfig::tracking` | 保持细粒度跟踪策略能力 |
| `config::PipelineConfig::expert.lifecycle` | `RadarPolicyConfig::lifecycle` | 生命周期参数全部进入 `policy` |
| `config::PipelineConfig::association` | `RadarPolicyConfig::tracking` | 与跟踪行为强绑定，收敛到 policy 域 |
| `model::RadarOrientationConfig` | `RadarMissionConfig` | 工作子模式、扫描中心、驻留中心、波束宽度均视为 mission/runtime domain |
| `environment::EnvironmentDefaultConfig` | `RadarEnvironmentConfig` | 直接保留为环境域 |

### Builder 映射

| 当前入口 | 目标入口 | 迁移原则 |
| --- | --- | --- |
| `RadarSessionConfigBuilder` | `RadarSessionConfigBuilder` | 保留名称，但输出新结构，不再向外暴露 `expert` |
| `RadarExpertSessionConfigBuilder` | `RadarDetailedSessionConfigBuilder` | 保留双入口思想，但公开概念从 expert 改为 detailed |
| `presets::MakeDefaultRadarSessionConfig()` 等 | 同名或等价 preset 工厂 | 直接返回新 `RadarSessionConfig` |

### Runtime Patch 映射

| 当前字段 | 目标位置 | 迁移原则 |
| --- | --- | --- |
| `has_pipeline_config/pipeline_config` | 删除 | 不再允许对外整块提交内部 `PipelineConfig` |
| `has_work_sub_mode/work_sub_mode` | `mission.work_mode` 或快捷叶子 | 保留运行时切换能力 |
| `has_scan_center_deg` | `mission.scan_center_deg` 或快捷叶子 | 保留 |
| `has_dwell_center_deg` | `mission.dwell_center_deg` 或快捷叶子 | 保留 |
| `has_commanded_beamwidth_deg` | `mission.commanded_beamwidth_deg` 或快捷叶子 | 保留 |
| `has_commanded_beamwidth_enabled` | `mission.commanded_beamwidth_enabled` 或快捷叶子 | 保留 |
| `has_environment_runtime_config` | `environment` 整块或环境补丁 | 保留环境热更新能力，但改成领域语言 |

## 分阶段实施步骤

### 阶段 1：重塑公开配置骨架

- 新增 `RadarHardwareConfig`、`RadarMissionConfig`、`RadarPolicyConfig` 及其子配置类型。
- 将当前 `RadarSessionConfig` 从 `pipeline_config + environment_default_config` 改为 `hardware + mission + policy + environment`。
- 保留 `airborne_radar_config.hpp` 作为统一入口，但聚合的新头改为领域类型。
- 将 `PipelineConfig.h` 从主公开合同降级；若短期内仍保留文件，则仅作为内部过渡层使用，不再作为对外推荐入口。

完成判据：

- 外部可以仅通过新 `RadarSessionConfig` 表达当前所有公开能力。
- 新结构不再要求调用方理解 `expert` 或 `pipeline` 才能配置完整雷达。

### 阶段 2：重构 Builder 体系

- 重写 `RadarSessionConfigBuilder`，输入 profile/preset 后直接生成新 `RadarSessionConfig`。
- 以 `RadarDetailedSessionConfigBuilder` 替换 `RadarExpertSessionConfigBuilder`。
- 详细 builder 要覆盖当前 expert builder 的能力，包括探测、波束、跟踪、生命周期、环境各域。
- 所有 builder 注释、示例、README 中的术语统一切换为 `hardware / mission / policy / environment`。

完成判据：

- 现有 `RadarSessionConfigBuilder` 能覆盖任务级/语义级构造场景。
- 新 `RadarDetailedSessionConfigBuilder` 能完整替换当前 expert builder 的细粒度使用路径。

### 阶段 3：重接解析与运行时路径

- 在 session composition 路径中新增从新 `RadarSessionConfig` 到内部运行态的映射。
- `RuntimeConfigResolver.cpp` 改为处理新 `RadarRuntimeConfigPatch`。
- 明确 runtime patch 规则：
  - 先应用整块域覆盖
  - 再应用叶子覆盖
  - 非有限数值直接拒绝，整次 patch 不生效
- 内部仍可保留 `PipelineConfig` 作为运行时装配结构，但不再由外部直接提交。

完成判据：

- `RadarSessionFactory::Create(...)` 能从新配置壳正确构造内部流水线。
- `ApplyRuntimeConfig(...)` 仅暴露业务允许的热更新能力。

### 阶段 4：更新公共文档与消费者用例

- 更新 `include/1q/airborne_radar/config/README.md`，移除对 `expert` 作为对外主模型的强调。
- 更新 `tests/consumer/ar_session_consumer.cpp`，全部切换到新配置 API。
- 更新 `tests/contract/public_headers_smoke_test.cpp` 中机载雷达相关用例。
- 更新任何直接依赖旧公开配置结构的 integration tests。

完成判据：

- consumer 示例不再出现 `pipeline_config.expert` 路径。
- 公开头冒烟测试按新命名和分层通过。

### 阶段 5：清理旧公开入口

- 删除或下线旧的 `RadarExpertSessionConfigBuilder`。
- 删除旧公开注释、示例、README 中对 `pipeline/expert` 的主路径描述。
- 移除所有只为兼容旧外部接口而保留的桥接层。

完成判据：

- 仓库内不再存在“新旧双轨并存”的长期状态。
- 外部推荐配置方式唯一且清晰。

## 测试与验收标准

### 配置类型层

- 新 `RadarSessionConfig` 必须完整覆盖当前 `RadarSessionConfig + PipelineConfig + EnvironmentDefaultConfig` 的公开表达能力。
- `RadarDetailedSessionConfigBuilder` 必须覆盖当前 `RadarExpertSessionConfigBuilder` 可调节的公开参数。
- 新 `RadarRuntimeConfigPatch` 只能暴露确实允许热更新的字段，不允许继续直接提交整套内部 `PipelineConfig`。

### 行为层

- session 初始化路径能够从新配置壳正确映射到内部 signal pipeline、decision pipeline 与 environment service。
- runtime patch 满足“先整块、后叶子”的优先级规则。
- 非有限数值或非法热更新字段组合会被整体拒绝，不产生部分生效。

### 需要更新和回归的测试

- `tests/consumer/ar_session_consumer.cpp`
- `tests/contract/public_headers_smoke_test.cpp`
- `tests/integration/ar_session_test.cpp`
- 任何直接断言 `pipeline_config.expert` 或旧 builder 输出结构的测试

### 验收标准

- 选定 preset 的 configure/build/test 在约定 preset 下通过。
- 机载雷达公开配置入口在命名和分层上与 EOS/ESR 目标架构同构。
- 新文档、示例和测试不再把内部流水线结构作为外部主语言。

## 风险、边界与默认决策

- 不做兼容层。旧公开结构一旦替换，调用方需要同步迁移。
- `PipelineConfig` 可以在内部装配路径暂时保留，但不作为外部入口长期存在。
- `hardware` 与 `policy` 的边界按以下规则固定：
  - 固有装备能力进入 `hardware`
  - 任务/算法/门限/调度策略进入 `policy`
- `mission` 域仅承载当前任务态、扫描态、指向态和允许运行期热更新的控制量。
- 环境域沿用现有 `EnvironmentDefaultConfig` / `EnvironmentRuntimeConfigPatch` 的能力，但公开命名统一为 `environment`。
