# Input Surface 枝干修剪与公开输入去镜像化策略方案

## 0. 文档定位

本文不是替代 [input_surface_unification_refactor_plan.md](/Users/aurora/Code/1q/input_surface_unification_refactor_plan.md)，而是它的后续收尾方案。

前一轮已经完成的事情是：

- 三模块统一了 `CycleInput { cycle_index, dt_sec, platform_pose, scene, environment }` 骨架
- 三模块建立了 `session/*SceneTypes.h` 这套公开输入语言
- AR/EOS/ESR 在命名上已从 `TargetFeature` / `EmitterTruthState` 这类内部语言中抽离

当前这一轮要解决的，不是再推翻这套规范，而是继续把规范内部残留的“旧实现味道”修干净。

一句话概括：

- 保留已经确定的公开输入壳
- 修剪壳里仍然带着内部模型痕迹的字段、重复结构和 include 暴露

## 1. 这轮到底要解决什么

当前最典型的问题是 AR：

- [RadarSceneTypes.h](/Users/aurora/Code/1q/include/1q/airborne_radar/session/RadarSceneTypes.h) 已经是公开输入类型
- 但它的字段和 [TargetFeature.h](/Users/aurora/Code/1q/include/1q/airborne_radar/model/TargetFeature.h) 仍然几乎一一对应
- 结果就是：名字换了，但内容还是内部处理语言

同类问题在 ESR 也存在：

- `EsrSceneEmitter` 已经替代了公开口上的 `EmitterTruthState`
- 但结构上仍高度贴近内部 `EmitterTruthState`

EOS 的问题略有不同：

- 它不是“公开类型直接镜像内部 model”
- 而是 public input 里有重复字段簇，尤其是目标外观/辐射属性在多个 public 结构里重复展开

所以这轮的目标不是“继续造新壳”，而是做三件事：

1. 让公开输入类型真正只表达外部事实
2. 让内部模型承担派生量、补全量、加工量
3. 让 public include 面不再把内部模型继续扩散给外部调用方

## 2. 核心判断标准

### 2.1 什么叫允许相同

如果一个字段本来就是外部世界的客观事实，那么它在公开输入层和内部模型层同时存在是正常的。

例如：

- 目标 ID
- 位置
- 速度
- RCS
- 发射频率
- 发射功率

这些字段“值相同”不是问题，因为它们本来就应该从外部一路传进内部。

### 2.2 什么叫必须分开

只要一个字段属于下面这些类别，就不应留在公开输入层：

- 派生量
- 补全量
- 归一化量
- 缓存态
- 跟踪态
- 真值/特征/解析结果这类内部阶段语言

例如：

- `current_track_speed`
- `current_track_*`
- `truth`
- `feature`
- `resolved`

### 2.3 这轮不是追求“长得完全不一样”

公开输入结构和内部模型结构不需要为了区分而强行做成完全不同。

正确目标是：

- 公开层只说“外部事实”
- 内部层在这些事实之上增加“内部加工数据”
- 两层之间有清晰、单向、集中化的转换

所以“70% 字段重合”可以接受，但“公开类型直接说内部阶段语言”不可以接受。

## 3. 总体策略

### 3.1 保留什么

这轮明确保留：

- `CycleInput.scene` / `CycleInput.environment` 统一骨架
- `session/*SceneTypes.h` 作为公开输入契约
- `RadarSceneTarget / EosSceneTarget / EsrSceneEmitter` 这套公开实体命名

### 3.2 删除什么

这轮明确要继续删除：

- 公开输入字段里的内部阶段语言
- 公开输入里的纯派生字段
- examples/tests/consumer 中把内部 model 当公开输入便利类型的写法
- public include 中不必要的内部 model 暴露

### 3.3 下沉什么

以下内容应继续下沉回 `src/` 或内部 model：

- 速度模长等派生量计算
- 公开输入到内部模型的补全/归一化
- 真值态与场景输入之间的内部桥接
- 仅服务于内部运行和测试便利的转换函数

## 4. AR 策略

### 4.1 AR 当前核心问题

AR 不是“没有独立公开输入类型”，而是“已经有独立公开输入类型，但内容仍然像 `TargetFeature` 的翻版”。

这会带来三个后果：

1. 外部调用方仍会被内部处理语言影响
2. `TargetFeature` 一旦调整，`RadarSceneTarget` 就容易被迫跟着调整
3. tests/examples 容易继续沿用 `TargetFeature -> RadarSceneTarget` 这种反向桥接，导致新规范名义成立、实际上未站稳

### 4.2 AR 的处理原则

`RadarSceneTarget` 只保留“外部目标事实”。

建议保留的字段类别：

- `external_target_id`
- 位置
- 速度
- RCS
- 起伏模型
- 明确的输入有效性标志（如果内部确实需要调用方显式表达）

建议删除或迁出的字段类别：

- `current_track_speed`
- 任何 `current_track_*` 命名
- 任何只能由内部计算得到的派生量

### 4.3 AR 的推荐调整方向

本轮不追求先改最终代码细节，但方向要明确：

- `current_track_velocity_x/y/z` 改成中性的速度命名
- `current_track_rcs` 改成中性的 `rcs`
- `current_track_speed` 从公开输入层删除
- `TargetFeature` 保留为内部工作数据，允许继续拥有算法侧字段

也就是说：

- `RadarSceneTarget` 是“用户提交的目标事实”
- `TargetFeature` 是“内部算法工单”

两者基础事实字段可以重合，但工单里的派生和内部阶段信息不能继续泄漏到用户提交表。

### 4.4 AR 代码落点

AR 的单向转换建议集中在 session/runtime 内部，而不是分散在 tests/examples。

候选落点：

- `src/airborne_radar/session/`
- `src/airborne_radar/runtime/components/`

要求：

- 公开头只暴露 `RadarSceneTarget`
- `RadarSceneTarget -> TargetFeature` 的转换集中在内部实现
- tests/examples 优先直接构造 `RadarSceneTarget`
- 不再让 contract test 默认先构造 `TargetFeature` 再转回来

## 5. ESR 策略

### 5.1 ESR 当前核心问题

ESR 的命名整改已经做了，但 `EsrSceneEmitter` 和 `EmitterTruthState` 在数据形态上仍高度相似。

这本身不一定是错，因为辐射源输入很多字段天然就是事实字段。

真正的问题是：

- `EmitterTruthState.h` 仍然在公开使用面出现得过重
- 一些 public include / tests / examples 还会默认把内部真值类型当成主视角

### 5.2 ESR 的处理原则

ESR 这轮不以“强行让 `EsrSceneEmitter` 和 `EmitterTruthState` 长得不一样”为目标。

ESR 更合理的目标是：

- 让 `EsrSceneEmitter` 成为唯一公开输入视角
- 让 `EmitterTruthState` 退回内部仿真/环境/回放语义
- 继续压缩 public include 和公开调用面上对 `EmitterTruthState.h` 的依赖

## 6. EOS 策略

### 6.1 EOS 当前核心问题

EOS 当前的主要问题不是“内部 model 污染公开输入”，而是 public input 内部自己还有字段簇重复。

典型例子是：

- `EosSceneTarget` 自己有一套外观/辐射字段
- `EosExternalInputAdapter.h` 里又定义了 `EosTargetAppearance`

### 6.2 EOS 的处理原则

EOS 应优先做的是公共字段簇收拢，而不是再新建一层新壳。

推荐方向：

- 将目标外观/辐射属性整理为公共组成部分
- `EosSceneTarget` 通过组合复用，而不是继续平铺复制
- public input 里的“同一语义多处展开”要收成一个权威结构

## 7. 执行顺序

### 第一阶段：AR 先收口

目标：

- 明确 `RadarSceneTarget` 的最终事实字段集
- 去掉公开层派生量和内部阶段命名
- 让 tests/examples 从 `TargetFeature` 视角切回 `RadarSceneTarget`

原因：

- AR 当前镜像问题最重
- AR 一旦收干净，这轮策略的判断标准就基本立住了

### 第二阶段：ESR 收缩内部真值暴露

目标：

- 保持 `EsrSceneEmitter` 作为公开事实输入
- 缩减 `EmitterTruthState.h` 在公开使用面上的存在感
- 把真值语义收回内部场景/回放/仿真层

### 第三阶段：EOS 去重复字段簇

目标：

- 收拢 `EosTargetAppearance` 与 `EosSceneTarget` 的重复内容
- 让 EOS public input 结构更紧凑、更清晰

## 8. 文档与测试同步要求

这轮属于“公开输入语义继续收敛”，所以文档和测试必须同步，不允许只改实现。

至少要同步：

- 相关 public headers 注释
- examples
- contract tests
- integration tests
- `tests/contract/check_public_api_boundary.cmake`
- `tests/contract/public_headers_smoke_test.cpp`
- 如有必要，补一份当前方案执行状态说明到总计划文档

## 9. 验收标准

完成后应满足以下标准：

1. `session/*SceneTypes.h` 仍然是公开输入权威入口，而不是被再次绕开
2. AR 公开输入不再出现明显内部阶段语言和纯派生字段
3. ESR 公开使用面明显减少对 `EmitterTruthState.h` 的依赖
4. EOS public input 中重复字段簇被收拢
5. examples/tests 不再默认以内部 model 作为公开输入构造入口
6. 相关 public header whitelist、smoke test、构建与目标测试通过

## 10. 决策摘要

这轮不是“删除新规范”，而是“让新规范真正成立”。

最关键的判断只有两条：

- 外部事实字段允许和内部模型重合
- 内部派生字段和内部阶段语言必须留在内部

按这个标准推进，才能既保住刚建立的统一输入规范，又把它从“换名字”继续推进到“换语义、换边界、换依赖面”。
