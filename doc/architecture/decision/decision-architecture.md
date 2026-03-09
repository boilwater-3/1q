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

#### 2.3.1 面向下一阶段的目标分类模块设计目标

从雷达目标分类、实际仿真与工程演进角度，目标分类模块的合理形态不应仅停留在当前的规则评分或简单仓储匹配，而应逐步演进为如下三段式结构：

1. 航迹历史与观测窗口准备：由跟踪域服务维护目标历史数据，而不是由分类器自行保存状态。
2. 分类特征提取：从历史窗口中提取可稳定复用的统计特征、机动特征与观测质量特征。
3. 分类模型推断：基于统一特征向量执行类别识别、置信度评估与拒识判定。

该拆分方式的核心原因是：分类器属于决策域中的推断节点，不应同时承担航迹历史管理与目标生命周期维护职责。

#### 2.3.2 建议输入

建议目标分类模块的输入至少覆盖以下信息：

- `RCS` 序列，而不是单点 `RCS`。
- 速度矢量，而不是单一速度标量。
- 加速度向量或至少包含横向加速度分量。
- 距离、高度、爬升率等观测几何信息。
- 航迹历史窗口，用于提取时序统计与机动特征。
- 观测质量指标，例如有效点数、信噪比区间、特征缺失情况。

说明：距离和高度更适合作为观测条件或特征质量修正因子，而不是直接作为目标类别本体特征参与硬判决。

#### 2.3.3 建议特征提取流程

建议将特征提取独立为可复用组件，例如 `TargetFeatureExtractor`，并输出统一的 `ClassificationFeatureVector`。推荐的特征分组如下：

1. `RCS` 统计特征
  - 均值
  - 方差
  - 起伏系数
  - 稳定性指标
2. 运动统计特征
  - 平均速度
  - 最大速度
  - 平均加速度
  - 爬升率
3. 机动特征
  - 横向加速度
  - 转弯率
  - 航向变化率
  - 机动性评分
4. 观测质量特征
  - 特征样本数
  - 观测窗口完整度
  - 当前观测质量分级

工程约束上，分类器不应直接从原始点迹临时拼装这些特征，而应消费经历史窗口聚合后的稳定摘要。

#### 2.3.4 分类模型合理性审查

目标分类设计中提出的“遍历类别 -> 高斯分布似然 -> 贝叶斯后验 -> 归一化 -> 取最大概率类别”方案，可作为第一阶段的基线模型，但应明确其适用边界：

1. 合理之处
  - 结构简单，易于实现与调参。
  - 便于在仿真环境中快速建立第一版可运行分类闭环。
  - 输出天然具备概率解释，便于后续与决策阈值联动。
2. 局限性
  - `RCS`、速度、机动特征在现实中往往不满足条件独立假设。
  - 单高斯分布难以描述姿态变化、观测角变化和任务阶段变化导致的多峰分布。
  - 观测噪声、特征缺失与异常样本会显著放大误判风险。

因此，推荐将“高斯 + 贝叶斯”明确为 `V1` 基线模型，而不是长期固定方案。后续可以平滑替换为高斯混合模型、马氏距离判别、多模型投票或轻量学习模型。

#### 2.3.5 置信度与拒识机制

仅输出“最大后验概率”作为置信度并不足以支撑工程应用。推荐的分类结果应至少包含：

- `Top-1` 类别
- `Top-1` 概率
- `Top-2` 概率间隔
- 分类熵或分散度指标
- 是否拒识为 `Unknown`
- 产生该结果的特征向量摘要或模型版本信息

这类输出比单一数值更利于：

- 后续决策模块做阈值联动。
- 仿真环境下做误差分析与回放定位。
- 工程上做模型替换时的兼容演进。

#### 2.3.6 当前实现与目标设计的差距

当前实现仍属于“轻量基线”：

- 输入特征主要为速度、`RCS`、干扰标记与加速度标量。
- 分类逻辑以规则评分和可选仓储匹配为主。
- 输出结果尚未形成完整的概率分类结果对象。

这意味着本文档描述的完整分类流程，当前只能视为下一阶段目标，而不能表述为“已经落地完成”。

#### 2.3.7 推荐的工程演进路径

建议按以下顺序推进目标分类模块：

1. 扩展输入契约
  - 扩展 `TargetFeature` 或新增独立分类输入结构，纳入向量、历史窗口与观测质量字段。
2. 下沉历史管理
  - 保持航迹历史由 `TrackLifecycleManager` 或跟踪域服务管理，分类器只消费聚合结果。
3. 独立特征提取器
  - 新增分类特征提取组件，统一输出特征向量。
4. 落地 `V1` 概率分类器
  - 以高斯似然 + 贝叶斯后验为基线实现，并保留替换空间。
5. 增加评估与仿真验证
  - 输出混淆矩阵、拒识率、不同噪声条件下的退化曲线，而不是只验证单次样例。

建议的修正方案

先补输入载荷，而不是先写贝叶斯公式
建议把输入拆成两层：
单周期量测：RCS、位置、速度、加速度、距离、高度、观测质量
航迹窗口特征：最近 N 帧历史、统计量、转弯率、横向过载、稳定性指标
也就是说，分类器不要直接吃原始点，而是吃“窗口化特征摘要”。
这部分更接近跟踪域服务输出，而不是决策层自己临时拼。

特征提取单独成模块
建议新增一个特征聚合器，例如：
TrackFeatureWindow
TargetFeatureExtractor
ClassificationFeatureVector
职责分开：

TrackLifecycleManager 维护航迹历史
FeatureExtractor 从历史窗口提分类特征
TargetClassifier 只做模型推断
这样领域边界最干净。

先用“高斯基线模型”，但明确它只是 V1
第一版可以保留：
每类参数化分布
先验概率
贝叶斯后验
归一化输出



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
- 当前目标分类仍未形成完整的“历史窗口 -> 特征提取 -> 概率分类 -> 拒识评估”闭环。
- 当前 `TargetCategory` 更接近轻量标签载体，后续宜演进为包含类别、置信度、间隔与证据摘要的分类结果对象。

## 6. 图示

- 架构图：`doc/architecture/decision/decision-architecture.puml`
- 周期流程图：`doc/architecture/decision/decision-cycle-flow.puml`
