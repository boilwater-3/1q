# Decision 层架构设计（基于当前实现）

## 1. 设计目标

Decision 层负责在单个处理周期内完成“目标识别 -> 低截获控制（LPI）-> 抗干扰控制（ECCM）”的顺序决策，并输出统一的雷达控制命令列表。

当前实现以 `DecisionContext` 为数据载体，以责任链模式组织处理节点，强调：

- 逻辑解耦：各节点通过视图访问上下文，不直接相互依赖。
- 数据单向流动：输入特征进入决策链，输出命令由核心层统一下发。
- 可扩展性：分类特征向量支持按键扩展，不固定三维结构。

## 2. 模块边界与职责

### 2.1 `DecisionContext`

文件：`include/1q/airborne_radar/core/context/DecisionContext.h`

职责：

- 保存当前周期输入：`targets_features`。
- 保存中间结果：`target_classification_result`。
- 保存来源信息：`lpi_source_info`、`eccm_source_info`。
- 保存输出命令：`decision_commands`。
- 提供模块视图构造函数：
  - `CreateTargetClassifierView(...)`
  - `CreateLpiControllerView(...)`
  - `CreateEccmControllerView(...)`

### 2.2 决策责任链骨架

文件：`include/1q/airborne_radar/decision/pipeline/ITacticalProcessor.h`

职责：

- `ITacticalProcessor` 封装责任链串接和统一入口 `ProcessTactics(...)`。
- `TacticalProcessor<TView>` 通过 `ChainProcessorWithView` 桥接为“上下文 -> 视图 -> 节点处理”。
- 节点仅处理本视图可见数据，避免跨模块读写污染。

### 2.3 `TargetClassifier`

文件：

- `src/airborne_radar/decision/classifier/TargetClassifier.h`
- `src/airborne_radar/decision/classifier/TargetClassifier.cpp`

职责：

- 遍历所有目标特征并逐一分类。
- 可选使用 `IFeatureRepository` 做匹配分类。
- 仓储匹配通过阈值过滤：
  - `probability >= 0.55`
  - `distance <= 1.80`
- 匹配失败回退规则分类（速度/RCS/干扰加权评分）。
- 更新 `LpiSourceInfo.has_recon_platform`。

### 2.4 `LpiController`

文件：`src/airborne_radar/decision/lpi/LpiController.cpp`

职责：

- 只读消费 `LpiSourceInfo`。
- 当 `has_recon_platform == true` 时，下发 LPI 命令。

当前状态说明：

- 已定义三个命令构造函数：`SET_LPI_POWER`、`SET_LPI_BEAMFORMING`、`SET_LPI_DWELL`。
- 当前实际仅 `push_back(BuildLpiPowerCommand())`，其余两个为预留未启用。

### 2.5 `EccmController`

文件：`src/airborne_radar/decision/eccm/EccmController.cpp`

职责：

- 只读消费 `EccmSourceInfo`。
- 当 `has_jamming_signal == true` 时，下发 5 条 ECCM 指令：
  - `ENABLE_SIDELOBE_CANCELLER`
  - `ENABLE_ADAPTIVE_BEAMFORMING`
  - `SET_AGILITY_FREQ`
  - `SET_ECCM_REJITTER`
  - `SET_ECCM_BURNTHROUGH_GAIN`

## 3. 关键数据模型

### 3.1 输入与分类结果

- `TargetFeatureList`：当前周期目标特征集合。
- `TargetCategoryList`：每个目标对应的分类结果。

### 3.2 来源信息

文件：`include/1q/airborne_radar/common/DecisionSourceInfo.h`

- `LpiSourceInfo`
  - `has_recon_platform`
- `EccmSourceInfo`
  - `has_jamming_signal`

当前 `EccmSourceInfo` 由 `DecisionContext` 在构造时通过 `IsJammingDetected()` 汇总得到。

## 4. 与核心层协作关系

文件：`src/airborne_radar/core/controller/RadarController.cpp`

协作流程：

1. 核心层构造 `DecisionContext(updated_features)`。
2. 调用 `decision_pipeline_.ProcessTactics(context)` 执行决策链。
3. 核心层统一遍历 `context.decision_commands` 下发命令。

Decision 层不直接调用硬件接口，保持逻辑层纯度。

## 5. 当前约束与后续演进点

- `TargetClassifier::UpdateLpiSourceInfo(...)` 仍使用高威胁标签近似映射侦察平台，属于临时策略。
- `LpiController` 当前只启用功率控制，波束与驻留命令尚未接入实际下发。
- `FeatureRepository` 的外部数据库拉取接口存在占位，实际驱动与 SQL 待确定。

## 6. 图示

- 架构图：`doc/architecture/decision/decision-architecture.puml`
- 周期流程图：`doc/architecture/decision/decision-cycle-flow.puml`
