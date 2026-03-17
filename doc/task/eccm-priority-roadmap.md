# ECCM 优先级任务拆解

## 1. 背景

当前仓库已经完成 `Environment -> Decision -> Signal` 的 ECCM 控制闭环，但最高优先级缺口仍然在“事实建模”而不是“性能优化”：

- 环境层已能输出干扰存在性与若干聚合事实。
- 决策层已能输出可叠加的 ECCM proposal。
- 信号层已能执行 `RadarControlProfile`。

现阶段主要问题是：干扰事实仍偏单源/聚合视角，不足以稳定支撑“按干扰类型、按干扰来源组合选择 ECCM”。

## 2. 优先级排序

### P0

1. 扩展多源干扰事实契约
2. 让 ECCM evaluator 基于多源事实做差异化策略选择

### P1

3. 把 `ControlReducer` 升级为正式冲突裁决器
4. 为控制真值增加保持周期、冷却和变更追踪

### P2

5. 把干扰类型/角域真正映射到探测、关联、Lifecycle
6. 统一频率捷变与 PRF 抖动对量测统计的传播

### P3

7. IMM 多线程优化与批量测试恢复

## 3. 各任务说明

### 3.1 扩展多源干扰事实契约

目标：

- `EnvironmentSnapshot` 支持多干扰源列表
- `EccmSourceInfo` 支持多干扰源列表
- 保留现有单值聚合字段，避免一次性推翻信号层

最小字段：

- 干扰类型
- 干扰功率
- J/S 或等效强度
- 频率重叠度
- PRF 锁定风险
- 入射角域
- 是否旁瓣入侵
- 置信度

验收条件：

- `RadarController` 能把环境层的多源事实透传到决策层
- 旧的单源字段和旧测试继续兼容

### 3.2 基于多源事实生成 ECCM 组合

目标：

- `SurvivabilityEvaluator` 不再只看一个聚合布尔量
- 能对压制式、欺骗式、转发式干扰生成不同 proposal 组合

验收条件：

- 多源输入可触发联合策略
- evaluator 单测覆盖多种干扰类型组合

### 3.3 ControlReducer 正式化

目标：

- 把 LPI / ECCM 冲突从固定限幅提升为配置化策略表
- 增加保持周期、冷却和变更痕迹

备注：

- reducer 的“细粒度策略表 + 来源追踪”不作为当前版本继续展开
- 该项下沉为下一版本任务，当前版本在域级 release / hold / cooldown / beam conflict 停止扩展

## 4. 本轮执行项

本轮已完成 P0 与 P1 的当前版本边界，并直接进入 P2：

1. 新增多源干扰事实契约
2. 在 `RadarController` 中完成映射
3. 让 `SurvivabilityEvaluator` 优先基于多源事实输出 proposal
4. 把 `SurvivabilityEvaluator` 从布尔开关升级为按干扰类型、强度、角域和置信度打分的策略优先级模型
5. 开始把 `ControlReducer` 升级为带域级 release / hold / cooldown 的正式归并器
6. 补齐对应测试
7. 进入 P2，把多源干扰类型/角域继续传播到物理探测、关联和 Lifecycle 统计

当前进度：

- 多源干扰事实契约：已完成
- 控制器映射：已完成
- 多源事实驱动 proposal：已完成
- 评分式策略优先级模型：已完成
- reducer 域级释放 / hold / cooldown / beam conflict：已完成
- P2 首轮传播实现：已完成
- P2 第二轮类型化关联/Lifecycle 容错：已完成
- P2 第三轮关联质量摘要与压力观测：已完成
- P2 第四轮决策层消费关联压力摘要：已完成
- P2 第五轮区分“关联受干扰”与“普通匹配变差”：已完成
- P2 第六轮补充探测压力摘要与模式解释：已完成

本轮不做：

- `ControlReducer` 的细粒度策略表 + 来源追踪
- IMM 并行优化

## 5. 相关文件

- `include/1q/airborne_radar/environment/IEnvironmentService.h`
- `include/1q/airborne_radar/environment/EnvironmentService.h`
- `src/airborne_radar/environment/EnvironmentService.cpp`
- `include/1q/airborne_radar/common/DecisionSourceInfo.h`
- `src/airborne_radar/core/controller/RadarController.cpp`
- `src/airborne_radar/decision/eccm/SurvivabilityEvaluator.cpp`
