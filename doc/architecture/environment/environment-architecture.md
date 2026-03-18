# Environment 层架构说明

## 1. 简介

Environment 层的职责是把场景、传播和干扰条件整理为单周期可消费的环境事实，并通过 `IEnvironmentService` 暴露给上层。

当前正式公共接口是：

- `environment::IEnvironmentService`
- `environment::EnvironmentCycleContext`
- `environment::EnvironmentSnapshot`
- `environment::EnvironmentService`
- `environment::EnvironmentSceneState`

其中 `EnvironmentSnapshot` 目前包含：

- `propagation_loss_db`
- `clutter_power_db`
- `jamming_detected`

当前环境层 MVP 已补齐两项基础语义：

- `BeginCycle(...)`：冻结单周期环境事实，确保同周期 `Signal/Decision` 读取同一份快照
- `EnvironmentSceneState`：以内存 API 维护待生效场景，并在下一周期提交

这已经足够驱动现有探测链和 ECCM 总开关，但对于更高保真的传播/干扰物理仍保持组合式近似。

## 2. 当前代码结构

```text
include/1q/airborne_radar/environment/
├── IEnvironmentService.h
└── EnvironmentService.h

src/airborne_radar/environment/
├── EnvironmentService.cpp
├── scene/SceneManager.*
└── simulation/PropagationModel.*
```

其中：

- `SceneManager` 负责 pending/active 场景切换
- `PropagationModel` 负责组合式传播损耗与杂波建模
- `EnvironmentService` 负责场景冻结、快照聚合和兼容旧配置接口

注意：当前代码中尚未拆出独立的 `InterferenceModel` 公共类。下面关于“干扰建模模块”的边界，属于基于现有 `EnvironmentService` 的推荐拆分方向。

## 3. 职责边界

### 3.1 环境层应该负责什么

| 职责 | 说明 |
|------|------|
| 传播损耗建模 | 例如大气衰减、地形/多径附加损耗 |
| 杂波强度建模 | 为探测链提供背景噪声项 |
| 干扰存在性建模 | 判断当前周期是否有干扰 |
| 干扰事实建模 | 强度、角域、频域重叠、时域相干性等 |

### 3.2 环境层不应该负责什么

| 不属于环境层的职责 | 原因 |
|--------------------|------|
| 选择旁瓣对消还是频率捷变 | 这是战术决策，不是物理事实 |
| 直接修改天线参数或 PRF | 这是信号层执行职责 |
| 决定 LPI 与 ECCM 谁优先 | 这是 reducer 的冲突裁决职责 |

一句话概括：环境层给出“外界条件”，但不下“应对命令”。

## 4. 干扰建模模块的推荐边界

建议把环境层内部的干扰建模抽象成以下三个概念，即便它们暂时不以独立类暴露：

| 子模块语义 | 输出 | 主要服务对象 |
|------------|------|--------------|
| 干扰检测 | 是否有干扰、置信度 | 决策层 ECCM 触发 |
| 干扰表征 | 强度、方向、频谱重叠、PRF 锁定风险 | 决策层策略选择 |
| 干扰注入 | 对探测链的等效噪声/损伤项 | 信号层探测执行 |

这样拆的好处是：

- 同一份干扰事实可以同时服务决策层和信号层。
- 决策层不需要知道物理细节公式。
- 信号层不需要知道策略为什么被选中。

## 5. 与决策层和信号层的数据流

```text
Scene / Propagation / Interference facts
  -> EnvironmentSnapshot
  -> RadarController
     -> DecisionInputFrame.environment_jamming_detected
     -> SignalPipeline.RunCycle(..., environment)
```

当前数据流中，环境层对两个上游模块的服务模式不同：

| 消费者 | 消费方式 | 关注点 |
|--------|----------|--------|
| Decision | 读取干扰事实摘要 | 是否需要启用 ECCM、启用哪些策略 |
| Signal | 读取环境噪声与损耗 | 探测链、误差链、关联和跟踪的运行参数 |

这条单向流应该保持稳定，不建议让 Decision 反向写 Environment。

## 6. 支撑各类 ECCM 技术所需的最小事实

如果要让 ECCM 从“总开关”演进为“按干扰类型组合策略”，环境层至少需要为每项技术提供下面这些事实维度。

| ECCM 技术 | 环境层最小事实 | 原因 |
|-----------|----------------|------|
| 旁瓣对消 | 干扰入射角域、旁瓣入侵判断 | 需要判断是否适合空域抑制 |
| 自适应波束形成 | 干扰方向、角度扩展、主旁瓣关系 | 需要决定是否值得形成零陷/缩波束 |
| 频率捷变 | 干扰频谱中心、带宽、频率重叠度 | 需要知道跳频是否有效 |
| 重频抖动 | 干扰锁定当前 PRF 的风险、相干/转发特征 | 需要知道 rejitter 是否有效 |
| 烧穿增益 | 干扰强度、目标价值、目标距离/JS 估计 | 需要知道是否值得用能量换生存性 |

这些是设计目标，不是当前 `EnvironmentSnapshot` 已有字段。

## 7. 推荐的事实分层

为了避免未来字段失控，建议把环境层输出分成三层语义：

### 7.1 全局环境事实

- 传播损耗
- 杂波功率
- 是否有干扰

### 7.2 干扰摘要事实

- 干扰强度等级
- 干扰方向/角域
- 与当前体制的重叠度
- 干扰类型标签

### 7.3 面向信号层的等效注入事实

- 等效 `jam_noise_w`
- 等效杂波增强项
- 是否需要扩大测量不确定性

这样可以保证：

- Decision 主要依赖摘要事实
- Signal 主要依赖等效注入事实

## 8. 当前实现与推荐演进

### 当前实现

- `EnvironmentService` 使用 `EnvironmentSceneState` 作为待生效场景输入
- `RadarController` 在每周期开始调用 `BeginCycle(...)`，冻结环境快照
- `PropagationModel` 以 `max(0, base + atmospheric + terrain)` 生成 `propagation_loss_db`
- `PropagationModel` 以 `max(0, clutter_power_db)` 生成 `clutter_power_db`
- `EnvironmentService` 聚合多源 jammer，取 `power_db` 最大者作为主摘要
- 用 `jammer_power_db >= threshold` 生成 `jamming_detected`
- `SignalPipeline` 再根据冻结后的环境事实和控制真值推导等效 `jam_noise_w`

### 一期完成基线（当前状态）

当前环境层已经达到最小可用实现，可作为后续二期开发基线，具体包括：

- 已有统一只读入口：`IEnvironmentService`
- 已有周期冻结语义：`BeginCycle(...)` 保证同周期 `Signal/Decision` 读取同一份快照
- 已有最小场景状态：`EnvironmentSceneState` 通过内存 API 管理传播、杂波和干扰源输入
- 已有最小场景管理：`SceneManager` 负责 pending/active 双态切换
- 已有最小传播模型：`PropagationModel` 负责组合式传播损耗与杂波输出
- 已有最小干扰建模：支持多源 jammer 输入、主摘要聚合和兼容旧版平铺配置
- 已打通信号层与决策层链路：`RadarController -> SignalPipeline / DecisionInputFrame`
- 已有测试兜底：覆盖周期冻结、场景提交、传播组合、兼容路径和跨层一致性

当前明确不包含的能力：

- 不按目标/按波束输出局部环境事实
- 不做距离相关传播修正
- 不做高保真大气衰减、多径、折射或地球曲率修正
- 不提供独立公开的 `InterferenceModel`
- 不支持文件场景格式、场景持久化和外部场景加载

### 推荐演进

1. 先扩展 `EnvironmentSnapshot` 的干扰事实颗粒度。
2. 再在 `RadarController` 中把其中面向决策的摘要映射为 `EccmSourceInfo`。
3. 保持 `RadarControlProfile` 仍是唯一进入信号层的控制真值。
4. 不要让决策层直接输出 `jam_noise_w`、`beamwidth_deg`、`frequency_hz` 这类执行参数。

### 二期开发指标

二期环境层的目标不是推翻当前实现，而是在当前一期基线之上扩展真实环境表达能力，建议按以下顺序推进：

1. 目标相关环境事实
   - 从全局周期快照扩展到与目标几何、距离、角域相关的局部环境事实
   - 让传播、杂波、干扰影响可随目标状态变化

2. 独立干扰建模模块
   - 从 `EnvironmentService` 内部聚合逻辑拆分出独立 `InterferenceModel`
   - 明确区分干扰检测、干扰表征、干扰注入三层职责

3. 更高保真传播模型
   - 引入距离相关传播修正
   - 逐步补充气象分项衰减、多径、折射和曲率等能力
   - 保持可退化到当前组合式模型，避免破坏现有主链路

4. 场景输入体系升级
   - 在保留内存 API 的前提下增加文件/外部配置加载
   - 定义场景格式、版本和校验规则

5. 测试基础设施增强
   - 持续扩展内存场景构造器/fixture
   - 增加目标相关环境事实和多模型传播/干扰组合的回归测试

## 9. 与当前代码的契合点

当前仓库已经具备下面这些设计基础：

- `IEnvironmentService` 是统一只读入口
- `DecisionInputFrame` 已为决策层保留 `environment_jamming_detected`
- `SignalPipeline::RunCycle()` 已在每周期采样环境
- `RadarControlProfile` 已能承载多种 ECCM 叠加策略

因此接下来最合理的增量不是重写链路，而是扩展环境事实，并继续维持“环境事实 -> 决策意图 -> 控制真值 -> 信号执行”的单向边界。
