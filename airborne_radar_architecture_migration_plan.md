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

需要特别指出的是，AR 与 EOS/ESR 的复杂度差异，不主要体现在公开配置字段数量，而体现在“公开配置如何落到运行骨架”这一层。当前仓库中的 AR 已经不是单纯的 `session -> pipeline` 关系，而是以下编排链：

- `RadarSession`
  - 持有 runtime state / pending runtime state
  - 控制 patch 提交、周期提交、失败回滚
- `RadarSessionCompositionRoot`
  - 负责把公开配置翻译为内部唯一真值，并装配 `context / signal pipeline / environment service / controller`
- `RadarController`
  - 负责单周期主干编排：`signal -> decision -> command -> output`
- `SignalPipeline`
  - 负责探测、关联、跟踪、轨迹/决策帧生产
- `EnvironmentService`
  - 负责环境冻结、场景状态、干扰敏感度与环境模型

因此，AR 的迁移计划不能照搬 EOS/ESR 的“先收敛配置类型，再平移 builder”模式，而必须显式处理下面三个问题：

- 四域公开模型之外，内部运行期的唯一真值到底是什么。
- `mission / policy / environment` 的热更新分别由谁消费，谁负责原子提交与失败回滚。
- `signal / decision / output / control` 这些子链路中，哪些属于公开架构的一部分，哪些只是内部装配细节。

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

## AR 适配原则

### 前置决策：公开 `config/` 目录结构需要调整

这件事需要先定，而且结论应当明确为：**需要调整**。

原因不是美观或命名统一，而是公开 `config/` 目录本身就是对外合同的一部分。对调用方来说，下面这些内容都会构成“公开架构认知”：

- 可以 include 的头文件名字
- `config/` 下是否存在 `expert/`、`semantic/`、`presets/` 这类子目录
- 模块统一入口头 `airborne_radar_config.hpp` 暴露了哪些概念
- 是否必须理解 `ExpertPipelineConfig`、`RadarOrientationConfig` 才能完成配置

只改结构体字段而不改目录结构，会产生一个表面四域、实际仍旧以 `expert/pipeline/orientation` 为公开心智模型的半迁移状态。这与当前改造目标冲突。

但这里的“需要调整”不等于“要把所有细粒度类型都直接删掉”。更准确的判定是：

- 公开主路径必须调整
- 历史 `expert/*` 子目录不能继续作为推荐公开入口
- `semantic/*`、`presets/*` 是否继续公开，取决于它们是否仍然服务于新的四域合同，而不是旧 `expert` 模型

### 原则 1：四域只解决公开合同，不直接替代内部运行骨架

AR 比 EOS/ESR 更复杂的根因是：外部看到的是配置模型，内部真正驱动行为的是周期执行骨架。四域模型应被视为“公开合同层”，而不是强行替换以下内部骨架：

- `session` 持有运行期唯一真值和补丁提交语义
- `composition root` 负责装配依赖与内部翻译
- `controller` 负责主循环编排与失败恢复
- `signal/decision/output/environment` 负责各自子域执行

迁移时应允许内部继续保留一个装配态聚合对象，但该对象不再暴露给外部，也不再默认等同于 `PipelineConfig`。

### 原则 2：先定义“内部装配态”，再谈四域到实现的映射

AR 当前实际依赖的运行真值不是单一对象，而是至少包含：

- `pipeline_config`
- `environment_scenario_config`
- `jamming_sensitivity_profile`
- `dwell_center_deg`

这说明 AR 已经天然存在一个“运行装配态”，只是当前名称和边界仍偏历史包袱。后续计划里应把它显式收敛为内部结构，例如：

- `RuntimeAssemblyConfig`
  - signal pipeline 所需配置
  - decision/control 所需配置
  - environment service 所需配置
  - runtime-only 状态量，例如 `dwell_center_deg`

四域模型的职责是生成这个内部装配态；`CompositionRoot`、`RuntimeConfigResolver`、`RadarSession` 的职责是消费并维护这个内部装配态。

### 原则 3：按“消费方”而不是按“字段归属”拆迁移

AR 中同一公开字段可能会同时影响多个内部子系统。典型例子：

- `mission.orientation`
  - 既影响 `signal pipeline` 的扫描/波束执行
  - 也影响 runtime patch 的叶子优先级与 dwell 状态
- `policy.beam_control`
  - 既影响扫描调度
  - 也隐式决定 mission 默认态
- `environment`
  - 同时影响 session 初始装配、周期冻结、干扰判定

因此 AR 的迁移必须按“谁消费这份配置”来拆：

- `Session` 负责 patch 生命周期与一致性
- `CompositionRoot` 负责初始翻译与依赖装配
- `SignalPipeline` 负责信号侧配置消费
- `RadarController` 负责 decision/control/output 编排，但不持有公开配置解释逻辑
- `EnvironmentService` 负责环境态消费与恢复

### 原则 4：把 mission 中的“任务态”和“运行态”拆开思考

AR 的 `mission` 不能简单理解成静态任务参数。它至少包含两种性质不同的内容：

- 会话级任务基线：例如默认工作子模式、默认扫描中心、默认波束宽度
- 周期级运行控制量：例如 runtime patch 注入的扫描中心、驻留中心、指令态波束宽度

因此计划中需要明确：

- `mission` 是公开域模型
- 但内部需要独立维护 `mission baseline` 与 `runtime overrides`
- `dwell_center_deg` 这类只在运行期存在的量，不必硬塞回会话静态配置结构

这也是 AR 比 EOS/ESR 更复杂的核心差异之一。

## 四域到内部编排骨架的适配矩阵

| 公开域 | 内部主要消费方 | 适配要求 |
| --- | --- | --- |
| `hardware` | `signal::detection`、部分 tracking 初始化 | 只表达固有能力，不混入任务阈值；由桥接层翻译到 signal 侧装配配置 |
| `mission` | `signal pipeline`、runtime state | 拆成初始 mission baseline 与 runtime override；叶子 patch 最终落到 runtime state |
| `policy` | `signal pipeline`、`decision/control` 协作边界 | 保持细粒度，但由内部装配态拆发给 beam/association/tracking/lifecycle，不让 controller 直接理解公开结构 |
| `environment` | `EnvironmentService`、周期冻结逻辑 | 初始装配与运行期 patch 共用同一语义模型，但消费路径分为 init/update 两条 |
| `session config` | `CompositionRoot` | 只负责“公开四域 -> 内部装配态”翻译，不直接承担周期级语义 |
| `runtime patch` | `RuntimeConfigResolver` + `RadarSession` | 负责“补丁 -> 下一版本运行装配态”的解析、校验、原子提交和回滚 |

## 对外 `config/` 目录结构决策

### 结论

对外 `include/1q/airborne_radar/config/` 目录结构应当调整，而且这是迁移前期就要确定的关键因素，不应等到实现末期再决定。

### 判断标准

是否需要调整，不看“现有文件还能不能继续复用”，而看下面三个标准：

- 调用方是否还能通过 include 路径直接接触旧主模型
- 模块统一入口头是否仍在强化旧术语
- 新增示例和 README 是否能够只使用新四域语言完成配置

只要上面任一项答案还是“不能”，目录结构就还没有完成迁移。

### 为什么 AR 这里必须动目录，而 EOS/ESR 没这个包袱

EOS/ESR 当前对外 `config/` 基本都是平铺公开头：

- `*HardwareConfig.h`
- `*MissionConfig.h`
- `*PolicyConfig.h`
- `*EnvironmentConfig.h`
- `*SessionConfig.h`
- `*RuntimeConfigPatch.h`
- `*RuntimeConfigBuilder.h`
- `*DetailedSessionConfigBuilder.h`

而 AR 当前公开目录仍然保留：

- `PipelineConfig.h`
- `expert/*`
- `semantic/*`
- `presets/*`
- 总入口头继续直接暴露 `semantic/*` 和 `RadarOrientationConfig`

这说明 AR 的外部 include 面仍然在告诉使用者：“真正的主模型还是 pipeline/expert/orientation，只是外面套了一层四域壳。”

### 推荐目标结构

建议把 AR 的公开 `config/` 目标结构收敛为与 EOS/ESR 同构的平铺主干：

- `RadarHardwareConfig.h`
- `RadarMissionConfig.h`
- `RadarPolicyConfig.h`
- `RadarEnvironmentConfig.h`
- `RadarSessionConfig.h`
- `RadarRuntimeConfigPatch.h`
- `RadarRuntimeConfigBuilder.h`
- `RadarSessionConfigBuilder.h`
- `RadarDetailedSessionConfigBuilder.h`
- `RadarWorkMode.h` 或等价 mission 基础类型头
- `airborne_radar_config.hpp`

同时按以下规则处理现有子目录：

- `expert/*`
  - 从公开主路径降级
  - 若内部仍需复用，可迁入内部实现头，或只作为非推荐细粒度参数载体存在
- `semantic/*`
  - 可以继续公开，但语义应服务于 `RadarSessionConfigBuilder`
  - 不应再成为“通往 expert 配置”的别名目录
- `presets/*`
  - 可以继续公开
  - 但返回值和文档必须完全站在新 `RadarSessionConfig` 上

### 不建议的方案

不建议采用下面这种折中方案：

- 维持现有 `config/` 目录主结构不动
- 仅新增若干四域结构体头
- 继续让 `expert/*`、`PipelineConfig.h`、`RadarOrientationConfig.h` 在统一入口头中占据主要地位

这个方案短期改动少，但长期会造成：

- 公开 API 心智模型分裂
- consumer 示例出现两套同等合法的 include 路径
- 文档很难明确“什么是主入口，什么只是遗留细节”
- 后续清理时又要做第二次公开破坏性迁移

### 建议的默认决策

如果当前要先拍板一个关键前提，我建议直接定为：

- `config/` 目录结构要改
- 改的是“公开主干结构”，不是要求一步删光所有历史细粒度类型
- 所有新文档、示例、builder、preset、统一入口头，只能站在新目录主干上说话
- 历史 `expert/*` 和 `PipelineConfig.h` 即使暂存，也不能继续占据公开推荐路径

## 目录迁移方案

### 1. 公开主干头文件清单

迁移完成后，`include/1q/airborne_radar/config/` 的公开主干应以平铺头文件为准。建议清单如下：

- `RadarHardwareConfig.h`
- `RadarMissionConfig.h`
- `RadarPolicyConfig.h`
- `RadarEnvironmentConfig.h`
- `RadarSessionConfig.h`
- `RadarRuntimeConfigPatch.h`
- `RadarRuntimeConfigBuilder.h`
- `RadarSessionConfigBuilder.h`
- `RadarDetailedSessionConfigBuilder.h`
- `RadarWorkMode.h`
- `RadarSessionConfigPresets.h`
- `airborne_radar_config.hpp`

如果某些基础类型暂时无法彻底从 `model/` 中抽离，也应满足一个更弱但必须达成的约束：

- 调用方仅通过上述主干头即可完成完整配置
- 不需要直接 include `PipelineConfig.h`
- 不需要直接 include `config/expert/*`
- 不需要直接 include `model/RadarOrientationConfig.h`

### 2. 现有头文件的分类处理

建议把现有公开头分成四类处理。

第一类：保留为公开主干

- `RadarSessionConfig.h`
- `RadarSessionConfigBuilder.h`
- `RadarDetailedSessionConfigBuilder.h`
- `RadarRuntimeConfigBuilder.h`
- `presets/RadarSessionConfigPresets.h`
- `airborne_radar_config.hpp`

但这里的“保留”不是原样不动，而是要改到不再把旧模型暴露为主入口。

第二类：拆分后保留公开语义，但迁移到主干

- 当前 `RadarSessionConfig.h` 中内联定义的：
  - `RadarHardwareConfig`
  - `RadarMissionConfig`
  - `RadarPolicyConfig`
- 当前 runtime patch 仅定义在 `RadarRuntimeConfigBuilder.h` 中
- 当前 mission 基础类型对 `RadarOrientationConfig` 的直接依赖

这类头应拆成独立公开主干头，原因是：

- 便于与 EOS/ESR 对齐
- 降低一个头聚合过多历史依赖
- 让调用方清晰感知四域边界

第三类：继续公开，但降级为“builder 配套语义材料”

- `semantic/AntennaProfiles.h`
- `semantic/DetectionProfiles.h`
- `semantic/LifecycleProfiles.h`
- `semantic/TrackingProfiles.h`
- `presets/RadarSessionConfigPresets.h`

这一类可以继续存在，但其定位要明确：

- 只服务于 `RadarSessionConfigBuilder` / presets
- 不再承担旧 expert 结构的语义入口职责
- 不应出现在“详细配置必须先理解这些 profile”这种文档叙述里

第四类：从公开主合同降级

- `PipelineConfig.h`
- `presets/PipelineConfigPresets.h`
- `expert/ExpertPipelineConfig.h`
- `expert/beam/*`
- `expert/detection/*`
- `expert/lifecycle/*`
- `expert/tracking/*`

这类头后续只有两种合理归宿：

- 迁为内部实现头
- 或者短期保留在 include 树中，但从统一入口头、README、consumer 示例、contract 边界清单中移除，明确标记为非推荐历史细粒度接口

如果项目坚持“不保留长期双轨兼容”，那么最终目标应是第一种，而不是第二种。

### 3. `airborne_radar_config.hpp` 的改造规则

统一入口头必须从“聚合旧语义子目录”改成“聚合四域主干头”。

迁移后它应优先暴露：

- `RadarHardwareConfig.h`
- `RadarMissionConfig.h`
- `RadarPolicyConfig.h`
- `RadarEnvironmentConfig.h`
- `RadarSessionConfig.h`
- `RadarRuntimeConfigPatch.h`
- `RadarRuntimeConfigBuilder.h`
- `RadarSessionConfigBuilder.h`
- `RadarDetailedSessionConfigBuilder.h`
- `RadarSessionConfigPresets.h`
- `RadarWorkMode.h`

它不应再直接暴露：

- `PipelineConfig.h`
- `config/expert/*`
- `model/RadarOrientationConfig.h`

`semantic/*` 是否继续从统一入口头导出，可以采用下面的默认策略：

- 若 `RadarSessionConfigBuilder` 的常见用法强依赖这些 profile，则保留导出
- 但导出顺序和文档叙事必须从属于四域主干，而不是与四域主干并列成为另一套主路径

### 4. `RadarSessionConfig.h` 的拆分策略

当前 `RadarSessionConfig.h` 同时承担了两件事：

- 定义四域子结构
- 暴露四域聚合会话结构

建议拆成：

- `RadarHardwareConfig.h`
- `RadarMissionConfig.h`
- `RadarPolicyConfig.h`
- `RadarEnvironmentConfig.h`
- `RadarSessionConfig.h`

其中 `RadarSessionConfig.h` 只聚合四域，不再继续承载所有子结构体定义。

这样做的收益是：

- include 依赖更清楚
- 四域边界在文件结构上也变得明确
- 后续把 `mission` 从 `RadarOrientationConfig` 逐步抽离时，不会牵动整个 `RadarSessionConfig.h`

### 5. `RadarRuntimeConfigPatch` 的落位策略

当前 runtime patch 直接放在 `RadarRuntimeConfigBuilder.h` 中，这会让“补丁模型”和“构造工具”绑在一起。

建议拆分为：

- `RadarRuntimeConfigPatch.h`
- `RadarRuntimeConfigBuilder.h`

其中：

- `RadarRuntimeConfigPatch.h` 只定义公开补丁合同
- `RadarRuntimeConfigBuilder.h` 只提供链式构造便利接口

这样更接近 EOS/ESR，也更方便后续让 `RuntimeConfigResolver` 只面向 patch 合同工作。

### 6. `mission` 基础类型的处理策略

`RadarMissionConfig` 当前本质上只是包了一层 `model::RadarOrientationConfig`。这里建议把决策拆成两个层级：

短期决策：

- 可以暂时继续复用 `RadarOrientationConfig`
- 但不应要求调用方直接 include `model/RadarOrientationConfig.h`
- 由 `RadarMissionConfig.h` 或 `RadarWorkMode.h` 对外承接 mission 相关基础类型

中期目标：

- 把真正属于 mission 公开合同的类型迁回 `config/`
- 将仅服务内部信号链路的方向/扫描实现细节留在 `model/` 或内部实现层

这一步不要求一次到位，但必须在文档里先把方向定清楚，否则 `mission` 会一直只是对旧 `orientation` 的别名包装。

### 7. README、边界测试和 consumer 示例要跟着一起迁

目录迁移不是只改头文件树，还必须同步改三类“强化公开心智模型”的地方。

第一类：README

- `include/1q/airborne_radar/config/README.md` 的目录树必须反映新主干
- 不再把 `PipelineConfig.h`、`ExpertPipelineConfig.h` 作为主路径讲解
- 如果还提到 `expert/*`，只能出现在历史说明或内部实现说明中

第二类：公开边界测试

至少需要改：

- `tests/contract/check_public_api_boundary.cmake`
- `tests/contract/public_headers_smoke_test.cpp`
- `tests/contract/ar_public_api_convenience_test.cpp`

这些测试目前仍把以下头视为公开合同的一部分：

- `PipelineConfig.h`
- `config/expert/*`
- `config/presets/PipelineConfigPresets.h`
- `model/RadarOrientationConfig.h`

如果不先改这里，代码实现再怎么迁，仓库的 contract 仍会把旧公开面锁死。

第三类：consumer/examples

至少需要回收：

- `tests/consumer/ar_session_consumer.cpp`
- `examples/ar/*`
- 直接 include `config/presets/*`、`model/RadarOrientationConfig.h`、`PipelineConfig.h` 的示例

这些示例必须改成只站在新公开主干上配置。

### 8. 可执行的阶段化落地顺序

建议按下面顺序推进，而不是同时全面重排：

1. 先确定新的公开主干清单和统一入口头策略。
2. 再拆 `RadarSessionConfig.h` 与 `RadarRuntimeConfigBuilder.h`。
3. 再修改 README、contract 边界测试、consumer 示例，锁定新的公开面。
4. 最后再清理 `PipelineConfig.h`、`expert/*`、`PipelineConfigPresets.h` 的公开残留。

这样做的好处是，先把“什么算公开合同”定死，再去做内部桥接和实现迁移，不会在实现中途反复返工 include 结构。

### 9. 建议写进计划中的决策结论

建议把目录相关的最终决策明确成下面这组句子：

- AR 对外 `config/` 目录结构需要调整。
- 调整目标不是简单新增四域类型，而是把四域头文件变成唯一公开主干。
- `semantic/*` 可以继续公开，但只作为 builder/preset 的语义材料。
- `PipelineConfig.h`、`PipelineConfigPresets.h`、`expert/*` 不再属于公开推荐入口。
- contract 测试、README、consumer 示例必须同步迁移，否则目录迁移不算完成。

## 建议的新目录树

建议目标公开树先收敛为下面这个形态：

```text
include/1q/airborne_radar/config/
|-- RadarHardwareConfig.h
|-- RadarMissionConfig.h
|-- RadarPolicyConfig.h
|-- RadarEnvironmentConfig.h
|-- RadarSessionConfig.h
|-- RadarRuntimeConfigPatch.h
|-- RadarRuntimeConfigBuilder.h
|-- RadarSessionConfigBuilder.h
|-- RadarDetailedSessionConfigBuilder.h
|-- RadarWorkMode.h
|-- RadarSessionConfigPresets.h
|-- airborne_radar_config.hpp
|-- semantic/
|   |-- AntennaProfiles.h
|   |-- DetectionProfiles.h
|   |-- LifecycleProfiles.h
|   `-- TrackingProfiles.h
`-- internal/                        (如必须保留公开 include 树中的过渡头，仅短期存在)
```

几点约束：

- `presets/` 目录可以直接平铺收口为 `RadarSessionConfigPresets.h`
- `PipelineConfigPresets.h` 不进入新树
- `expert/` 不进入新树
- 如果短期不得不保留过渡头，最多放到 `config/internal/` 这类明显非主路径的位置，并且不出现在统一入口头里

## 旧文件去向表

### 公开主干保留或拆分

| 现有文件 | 建议去向 | 处理方式 |
| --- | --- | --- |
| `config/RadarSessionConfig.h` | 拆为 `RadarHardwareConfig.h`、`RadarMissionConfig.h`、`RadarPolicyConfig.h`、`RadarEnvironmentConfig.h`、`RadarSessionConfig.h` | `RadarSessionConfig.h` 只做聚合；四域子结构独立成头 |
| `config/RadarRuntimeConfigBuilder.h` | 拆为 `RadarRuntimeConfigPatch.h` + `RadarRuntimeConfigBuilder.h` | 合同与构造器分离 |
| `config/RadarSessionConfigBuilder.h` | 保留 | 改为只依赖新主干头 |
| `config/RadarDetailedSessionConfigBuilder.h` | 保留 | 改为只依赖新主干头与必要语义头 |
| `config/airborne_radar_config.hpp` | 保留 | 改为只聚合新主干头 |
| `config/presets/RadarSessionConfigPresets.h` | 迁为 `config/RadarSessionConfigPresets.h` | 从子目录平铺到主干 |

### 继续公开，但降级为配套材料

| 现有文件 | 建议去向 | 处理方式 |
| --- | --- | --- |
| `config/semantic/AntennaProfiles.h` | 保留原位或后续平铺 | 继续公开，仅服务 builder |
| `config/semantic/DetectionProfiles.h` | 保留原位或后续平铺 | 同上 |
| `config/semantic/LifecycleProfiles.h` | 保留原位或后续平铺 | 同上 |
| `config/semantic/TrackingProfiles.h` | 保留原位或后续平铺 | 同上 |

这里建议短期先保留 `semantic/` 子目录，不把它和主干调整绑死在同一批次里。

### 从公开主合同降级

| 现有文件 | 建议去向 | 处理方式 |
| --- | --- | --- |
| `config/PipelineConfig.h` | 内部实现头或 `config/internal/PipelineConfig.h` | 从公开推荐入口移除 |
| `config/presets/PipelineConfigPresets.h` | 内部实现头 | 不再作为公开预设头 |
| `config/expert/ExpertPipelineConfig.h` | 内部实现头 | 不再列入 public contract |
| `config/expert/beam/*` | 内部实现头 | 不再列入 public contract |
| `config/expert/detection/*` | 内部实现头 | 不再列入 public contract |
| `config/expert/lifecycle/*` | 内部实现头 | 不再列入 public contract |
| `config/expert/tracking/*` | 内部实现头 | 不再列入 public contract |

## 新文件与旧依赖的对应关系

### `RadarHardwareConfig.h`

建议承接现有：

- `expert::DetectionConfig`
- 如果后续继续细拆，再逐步把 detection 下真正属于公开硬件合同的子类型迁出来

短期允许内部字段仍复用旧 detection 类型，但调用方不应再 include `config/expert/detection/*`。

### `RadarMissionConfig.h`

建议承接现有：

- `model::RadarOrientationConfig`
- `RadarWorkSubMode` 及 mission 相关基础类型

短期可以继续包裹旧 orientation 类型；中期再把 mission 合同类型逐步从 `model/` 迁回 `config/`。

### `RadarPolicyConfig.h`

建议承接现有：

- `expert::BeamControlConfig`
- `expert::AssociationConfig`
- `expert::TrackingConfig`
- `expert::LifecycleConfig`
- `expert::ImmConfig`

短期同样允许内部字段继续复用旧 expert 类型，但调用方不应直接 include `config/expert/*`。

### `RadarEnvironmentConfig.h`

建议承接现有：

- `environment::EnvironmentDefaultConfig`

这个文件本质上是给 AR 对外合同补一层显式命名，和 EOS/ESR 对齐。

### `RadarRuntimeConfigPatch.h`

建议承接现有：

- `RadarRuntimeConfigBuilder.h` 里的 `RadarRuntimeConfigPatch`

拆出后，builder 只依赖 patch 头，`RuntimeConfigResolver` 只面向 patch 合同。

### `RadarSessionConfigPresets.h`

建议承接现有：

- `config/presets/RadarSessionConfigPresets.h`

不建议继续保留在 `presets/` 子目录中，因为它已经是典型公开主干入口，而不是底层辅助材料。

## 对统一入口头的具体要求

迁移后的 [`airborne_radar_config.hpp`](/Users/aurora/Code/1q/include/1q/airborne_radar/config/airborne_radar_config.hpp) 应满足以下清单：

- 必须 include 新四域主干头
- 必须 include `RadarRuntimeConfigPatch.h`
- 可以 include `RadarRuntimeConfigBuilder.h`
- 可以 include `RadarSessionConfigPresets.h`
- 可以 include `semantic/*`
- 不能 include `PipelineConfig.h`
- 不能 include `config/expert/*`
- 不应直接 include `model/RadarOrientationConfig.h`

如果某个公开能力仍必须通过 `model/RadarOrientationConfig.h` 暴露，说明 `RadarMissionConfig.h` 还没有真正承接 mission 合同。

## 需要同步改动的仓库文件

### 第一批必须一起改

下面这些文件建议和目录主干迁移放在同一批次：

- [include/1q/airborne_radar/config/README.md](/Users/aurora/Code/1q/include/1q/airborne_radar/config/README.md)
- [include/1q/airborne_radar/config/airborne_radar_config.hpp](/Users/aurora/Code/1q/include/1q/airborne_radar/config/airborne_radar_config.hpp)
- [tests/contract/check_public_api_boundary.cmake](/Users/aurora/Code/1q/tests/contract/check_public_api_boundary.cmake)
- [tests/contract/public_headers_smoke_test.cpp](/Users/aurora/Code/1q/tests/contract/public_headers_smoke_test.cpp)
- [tests/contract/ar_public_api_convenience_test.cpp](/Users/aurora/Code/1q/tests/contract/ar_public_api_convenience_test.cpp)

原因是这几处共同定义了“仓库认为什么是公开合同”。

### 第二批紧跟着改

- [tests/consumer/ar_session_consumer.cpp](/Users/aurora/Code/1q/tests/consumer/ar_session_consumer.cpp)
- `examples/ar/*`
- 任何直接 include `PipelineConfig.h`、`config/expert/*`、`config/presets/PipelineConfigPresets.h` 的测试或示例

这一批负责把用户可见用法完全切到新主干。

### 第三批再回收内部引用

- `src/airborne_radar/session/SessionConfigBridge.h`
- `src/airborne_radar/session/RadarSessionCompositionRoot.h`
- `src/airborne_radar/signal/pipeline/config/SignalPipelineExecutionConfig.h`
- `include/1q/airborne_radar/extension/ISignalPipeline.h`

这一批重点不是改公开 include 面，而是把内部对 `PipelineConfig` 的依赖收回到内部装配边界。

## 建议的拆文件执行顺序

为了降低返工，建议按下面顺序实施：

1. 新增主干头：`RadarHardwareConfig.h`、`RadarMissionConfig.h`、`RadarPolicyConfig.h`、`RadarEnvironmentConfig.h`、`RadarRuntimeConfigPatch.h`、`RadarSessionConfigPresets.h`。
2. 让 `RadarSessionConfig.h`、`RadarRuntimeConfigBuilder.h`、`airborne_radar_config.hpp` 改为依赖这些新头。
3. 修改 README 和 contract 边界清单，先把“公开主干是什么”锁定。
4. 修改 consumer/examples，清理外部使用方式。
5. 最后再下沉 `PipelineConfig.h`、`PipelineConfigPresets.h`、`expert/*`。

这个顺序的关键是先锁公开合同，再动内部实现依赖；否则你们会不断在“旧头还算不算公开”这个问题上反复。

## 可直接拿来分任务的拆分包

### 包 1：公开主干成形

- 新增四域主干头
- 新增 `RadarRuntimeConfigPatch.h`
- 新增平铺版 `RadarSessionConfigPresets.h`
- 修改 `RadarSessionConfig.h`
- 修改 `RadarRuntimeConfigBuilder.h`
- 修改 `airborne_radar_config.hpp`

### 包 2：公开边界重置

- 修改 `config/README.md`
- 修改 `check_public_api_boundary.cmake`
- 修改 `public_headers_smoke_test.cpp`
- 修改 `ar_public_api_convenience_test.cpp`

### 包 3：外部使用方式迁移

- 修改 `tests/consumer/ar_session_consumer.cpp`
- 修改 `examples/ar/*`
- 清理对旧公开头的 include

### 包 4：内部过渡结构下沉

- 回收 `PipelineConfig.h`
- 回收 `PipelineConfigPresets.h`
- 回收 `expert/*`
- 收紧 `SessionConfigBridge / CompositionRoot / RuntimeConfigResolver` 对旧头的依赖范围

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

### 阶段 1：冻结内部装配边界，再收敛公开骨架

- 明确内部唯一真值不再直接命名为公开 `PipelineConfig`，而是定义清楚 AR 运行装配态的边界。
- 明确该内部装配态至少覆盖：
  - signal pipeline 配置
  - environment 初始配置
  - jamming sensitivity
  - runtime-only 控制量，例如 `dwell_center_deg`
- 在这个边界稳定后，再由 `RadarSessionConfig` 对外表达 `hardware + mission + policy + environment`。
- 保留 `airborne_radar_config.hpp` 作为统一入口，但聚合的新头改为领域类型。
- 将 `PipelineConfig.h` 从主公开合同降级；若短期内仍保留文件，则只允许出现在内部翻译路径。

完成判据：

- 外部可以仅通过新 `RadarSessionConfig` 表达当前所有公开能力。
- 内部已经有清晰的装配态边界，后续重构不再把公开四域直接泄漏到 `controller/signal/environment` 各层。
- 新结构不再要求调用方理解 `expert` 或 `pipeline` 才能配置完整雷达。

### 阶段 2：重构 Builder，但只让 Builder 生产公开模型

- 重写 `RadarSessionConfigBuilder`，输入 profile/preset 后直接生成新 `RadarSessionConfig`。
- 以 `RadarDetailedSessionConfigBuilder` 替换 `RadarExpertSessionConfigBuilder`。
- 详细 builder 要覆盖当前 expert builder 的能力，包括探测、波束、跟踪、生命周期、环境各域。
- builder 内可以继续复用历史 expert 语义类型做参数容器，但不能让 `expert` 再成为对外主概念。
- 对于 `BeamEditor` 这类同时写 mission/policy 的编辑器，要明确哪些字段是在同步“任务基线”，哪些字段是在设置“策略默认值”。
- 所有 builder 注释、示例、README 中的术语统一切换为 `hardware / mission / policy / environment`。

完成判据：

- 现有 `RadarSessionConfigBuilder` 能覆盖任务级/语义级构造场景。
- 新 `RadarDetailedSessionConfigBuilder` 能完整替换当前 expert builder 的细粒度使用路径。
- builder 产物不再被视为内部运行真值本身，而只是输入 `CompositionRoot` 的公开合同。

### 阶段 3：重接 `CompositionRoot / RuntimeConfigResolver / RadarSession`

- `SessionConfigBridge` 或等价桥接层负责从新 `RadarSessionConfig` 生成内部装配态，不再只是简单拼回旧 `PipelineConfig`。
- `RadarSessionCompositionRoot` 只消费内部装配态来装配：
  - `IRadarContext`
  - `ISignalPipeline`
  - `IEnvironmentService`
  - `RadarController`
- `RuntimeConfigResolver.cpp` 改为处理“补丁 -> 下一版本内部装配态”的解析，而不是面向历史 `PipelineConfig` 做字段覆盖。
- `RadarSession` 继续作为 runtime patch 的唯一入口，并维持 pending/commit/rollback 语义。
- 明确 runtime patch 规则：
  - 先应用整块域覆盖
  - 再应用叶子覆盖
  - 非有限数值直接拒绝，整次 patch 不生效
- `RadarController` 不解释公开四域，也不承接配置翻译职责；它只消费已经收敛完成的运行装配态和运行期快照。

完成判据：

- `RadarSessionFactory::Create(...)` 能从新配置壳正确构造内部运行骨架。
- `ApplyRuntimeConfig(...)` 仅暴露业务允许的热更新能力。
- patch 应用、周期提交、失败回滚的职责边界在 `RadarSession` 与 `RadarController` 之间清晰稳定。

### 阶段 4：按骨架分层回收内部直连点

- 检查 `signal pipeline`、`controller`、`environment service` 是否仍有代码直接依赖公开四域或历史 `expert/pipeline` 命名。
- 将这些直连点回收到桥接层或内部装配态，避免内部模块直接依赖公开合同。
- 明确 `decision/output/control` 不新增对公开配置类型的直接依赖。

完成判据：

- AR 内部模块依赖方向稳定为：公开合同 -> bridge/composition -> 内部装配态 -> 各执行模块。
- `RadarController`、`SignalPipeline`、`EnvironmentService` 不再各自偷偷解释公开配置。

### 阶段 5：更新公共文档与消费者用例

- 更新 `include/1q/airborne_radar/config/README.md`，移除对 `expert` 作为对外主模型的强调。
- 更新 `tests/consumer/ar_session_consumer.cpp`，全部切换到新配置 API。
- 更新 `tests/contract/public_headers_smoke_test.cpp` 中机载雷达相关用例。
- 更新任何直接依赖旧公开配置结构的 integration tests。

完成判据：

- consumer 示例不再出现 `pipeline_config.expert` 路径。
- 公开头冒烟测试按新命名和分层通过。

## 阶段计划（执行排期版）

以下排期按连续 6 个工作周估算，可按人力线性压缩或拉伸；关键是阶段门槛顺序不变。

### 里程碑总览

| 里程碑 | 周期 | 目标 | 对应拆分包 |
| --- | --- | --- | --- |
| M0 决策冻结 | W0（2-3 天） | 冻结公开 `config/` 主干清单和旧文件去向 | 准备阶段 |
| M1 公开主干成形 | W1 | 新增主干头并改统一入口头 | 包 1 |
| M2 公开边界重置 | W2 | 更新 contract/README，锁定公开面 | 包 2 |
| M3 外部用法迁移 | W3 | consumer/examples 全量切到新主干 | 包 3 |
| M4 内部装配迁移 | W4-W5 | 回收内部对旧公开头的依赖 | 包 4 + 阶段 3/4 |
| M5 清理与验收 | W6 | 清理遗留头并完成最终验收 | 阶段 5/6 |

### M0：决策冻结（W0）

输入：

- 当前迁移计划文档
- 现有公开头白名单与 contract 测试约束

输出：

- 冻结后的“新目录树 + 旧文件去向表”
- 明确“允许短期保留的过渡头”清单（如有）
- 阶段责任人和提交节奏（建议按里程碑分支）

门槛：

- 团队对以下 4 点达成一致：
  - `PipelineConfig.h` 不再属于公开推荐入口
  - `expert/*` 不再属于公开推荐入口
  - `semantic/*` 仅作为 builder/preset 语义材料
  - 统一入口头转向新主干

### M1：公开主干成形（W1）

输入：

- M0 冻结清单

输出：

- 新增主干头：`RadarHardwareConfig.h`、`RadarMissionConfig.h`、`RadarPolicyConfig.h`、`RadarEnvironmentConfig.h`、`RadarRuntimeConfigPatch.h`、`RadarSessionConfigPresets.h`
- `RadarSessionConfig.h` 改为聚合头
- `RadarRuntimeConfigBuilder.h` 与 patch 头解耦
- `airborne_radar_config.hpp` 只聚合新主干

门槛：

- 新主干头可以独立编译通过
- 统一入口头不再直接 include `PipelineConfig.h`、`config/expert/*`

### M2：公开边界重置（W2）

输入：

- M1 完成后的头文件布局

输出：

- 更新 [check_public_api_boundary.cmake](/Users/aurora/Code/1q/tests/contract/check_public_api_boundary.cmake)
- 更新 [public_headers_smoke_test.cpp](/Users/aurora/Code/1q/tests/contract/public_headers_smoke_test.cpp)
- 更新 [ar_public_api_convenience_test.cpp](/Users/aurora/Code/1q/tests/contract/ar_public_api_convenience_test.cpp)
- 更新 [README.md](/Users/aurora/Code/1q/include/1q/airborne_radar/config/README.md)

门槛：

- 新公开白名单与实际公开头一致
- 文档与 contract 测试不再强化旧术语主路径

### M3：外部用法迁移（W3）

输入：

- M2 锁定后的公开面

输出：

- `tests/consumer` 迁移完成
- `examples/ar` 迁移完成
- 对外示例不再 include 旧公开头

门槛：

- consumer 与 examples 仅依赖新主干配置头
- 不再出现 `pipeline_config.expert` 风格路径

### M4：内部装配迁移（W4-W5）

输入：

- M3 之后稳定的公开合同

输出：

- `SessionConfigBridge`、`CompositionRoot`、`RuntimeConfigResolver` 完成对“内部装配态”的对齐
- `SignalPipeline`、`RadarController`、`EnvironmentService` 的配置消费边界收敛
- 旧公开头在内部使用范围收敛到过渡边界

门槛：

- runtime patch 仍满足“先整块、后叶子”
- patch 提交、周期提交、失败回滚行为一致
- 内部依赖方向稳定为“公开合同 -> bridge/composition -> 内部装配态 -> 执行模块”

### M5：清理与验收（W6）

输入：

- M4 完成结果

输出：

- 删除或下沉旧公开残留：`PipelineConfig.h`、`PipelineConfigPresets.h`、`expert/*`（按最终决策执行）
- 清理遗留注释、README、示例术语
- 最终验收报告（公开面、行为、测试）

门槛：

- 不存在长期双轨公开路径
- 公开 API、文档、consumer、contract 测试一致

## 每阶段统一检查清单

每个里程碑结束都执行同一套检查，避免问题后移到最后一周：

1. 公开头检查：白名单与实际目录一致。
2. 语义一致性检查：README、统一入口头、consumer 示例叙事一致。
3. 行为检查：runtime patch 优先级和失败回滚语义未回归。
4. 构建测试检查：按约定 preset 完成 build 后再执行 ctest。

## 分支与合入建议

建议采用“里程碑分支 + 小步 PR”：

1. `codex/ar-config-m1-public-spine`
2. `codex/ar-config-m2-contract-boundary`
3. `codex/ar-config-m3-consumer-examples`
4. `codex/ar-config-m4-internal-assembly`
5. `codex/ar-config-m5-cleanup`

每个分支只做单里程碑范围内的改动，避免跨阶段混改导致回滚困难。

### 阶段 6：清理旧公开入口与历史桥接残留

- 删除或下线旧的 `RadarExpertSessionConfigBuilder`。
- 删除旧公开注释、示例、README 中对 `pipeline/expert` 的主路径描述。
- 移除所有只为兼容旧外部接口而保留的桥接层。
- 如果内部仍保留历史 `PipelineConfig`，则限制其只存在于内部装配实现，不能继续作为“事实上的对外模型”回渗到公共头文件和 consumer 示例。

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
- 周期执行失败时，runtime patch 的提交、环境冻结、controller 状态恢复保持一致，不出现部分提交。

### 需要更新和回归的测试

- `tests/consumer/ar_session_consumer.cpp`
- `tests/contract/public_headers_smoke_test.cpp`
- `tests/integration/ar_session_test.cpp`
- 任何直接断言 `pipeline_config.expert` 或旧 builder 输出结构的测试

### 验收标准

- 选定 preset 的 configure/build/test 在约定 preset 下通过。
- 机载雷达公开配置入口在命名和分层上与 EOS/ESR 目标架构同构。
- 新文档、示例和测试不再把内部流水线结构作为外部主语言。
- AR 特有的运行骨架复杂度已经在计划中有明确承接点，而不是被隐含地塞进“后续实现细节”。

## 风险、边界与默认决策

- 不做兼容层。旧公开结构一旦替换，调用方需要同步迁移。
- `PipelineConfig` 可以在内部装配路径暂时保留，但不作为外部入口长期存在。
- `hardware` 与 `policy` 的边界按以下规则固定：
  - 固有装备能力进入 `hardware`
  - 任务/算法/门限/调度策略进入 `policy`
- `mission` 域只承载公开层的任务态语言；内部是否拆成 baseline/override/runtime-only state，由内部装配态自行承担。
- 环境域沿用现有 `EnvironmentDefaultConfig` / `EnvironmentRuntimeConfigPatch` 的能力，但公开命名统一为 `environment`。
- AR 的重点不是把所有内部结构都改名成四域，而是保证四域公开模型与内部运行骨架之间只有一个清晰、稳定、可测试的翻译边界。
