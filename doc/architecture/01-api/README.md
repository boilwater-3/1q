# 01 — 公共 API 层

本层是面向外部调用方的门面封装，提供"易于正确使用、难以错误使用"的公共接口。

## 双层 API 设计

```text
┌──────────────────────────────────────────────────┐
│  Layer 1: RadarSession（一步到位门面）             │  ← 推荐入口
│  - Step(input) / StepWithResult(input, scene)    │
│  - ConfigPresets 预设工厂                         │
│  - TrackOutputQueries 查询工具                    │
│  - RadarInputValidation 输入校验                  │
│  - EnvironmentSceneBuilder 场景构造               │
└──────────────────────────────────────────────────┘
                      │
                      │ 需要自定义组件时向下穿透
                      ▼
┌──────────────────────────────────────────────────┐
│  Layer 2: RadarController + 接口注入              │  ← 高级扩展
│  - 自定义 ISignalPipeline                         │
│  - 自定义 ITacticalDecisionEngine                 │
│  - 自定义 IEnvironmentService                     │
│  - 自定义 IRadarContext                           │
└──────────────────────────────────────────────────┘
```

Layer 1 为 80% 的标准使用场景提供简洁路径，Layer 2 为 20% 的高级定制场景保留完整控制力。穿透边界由 [public-api-boundary.md](../public-api-boundary.md) 约束。

## 关键模块设计概述

### RadarSession — 高层门面

- **模式**：PIMPL（`std::unique_ptr<Impl>`），隐藏内部实现，保证二进制兼容性
- **执行模型**："一步一帧"——每次调用 `Step(input)` 驱动一个完整雷达处理周期并返回 `TrackOutputFrame`
- **Step 变体**：
  - `Step(RadarCycleInput)` → `TrackOutputFrame`（简单输出）
  - `Step(RadarCycleInput, EnvironmentSceneState)` → `TrackOutputFrame`（附带场景更新）
  - `StepWithResult(...)` → `RadarCycleResult`（聚合输出：轨迹帧 + 控制命令 + 关联质量指标）
- **容错**：不抛出异常；非法输入（`dt_sec ≤ 0`）返回尽力而为的结果
- **运行时配置更新**：`UpdateSignalPipelineConfig()`、`UpdateEnvironmentModelConfig()`、`SetJammingDetectionThresholdDb()`

### ConfigPresets + RadarSessionConfigBuilder — 配置体系

- **ConfigPresets**：预设工厂函数，提供开箱即用的调参起点
  - `MakeDetectionMissionRadarSessionConfig()` — 探测任务优化
  - `MakeTrackingMissionSignalPipelineConfig()` — 稳定跟踪优化
  - `MakeHighRobustnessSignalPipelineConfig()` — 高鲁棒性优化
- **RadarSessionConfigBuilder**：Builder 链式接口，基于预设进一步微调
  - 发射机参数（功率、频率、带宽、脉宽、PRF、损耗）
  - 天线参数（主波束增益、波束宽度）
  - 接收机参数（噪声系数、接收损耗）
  - 物理探测开关（雷达方程校验、探测余量、脉冲积累）
  - 跟踪参数（卡尔曼量测噪声标准差）
  - 生命周期管理（确认门限、丢失门限、丢失保留周期数）
  - `Build()` → `RadarSessionConfig`

### TargetFeatureBuilder / EnvironmentSceneBuilder — 输入构造

- **TargetFeatureBuilder**：按语义分步构造目标输入，避免位置参数误传
  - `Position(x, y, z)` → `Velocity(vx, vy, vz)` → `Rcs(rcs)` → `Build()`
  - 自动完成距离几何归一化
- **EnvironmentSceneBuilder**：链式构造环境场景状态
  - 传播损耗、大气衰减、地形反射、杂波功率
  - 按类型添加干扰源：噪声干扰 / 欺骗干扰 / 转发干扰

### TrackOutputQueries — 输出查询工具

纯函数集合，对 `TrackOutputFrame` 做只读过滤和统计：

- **映射构建**：`BuildTrackMapByExternalTargetId()`、`BuildTrackMapByAssociationKey()`
- **条件收集**：`CollectConfirmedTracks()`、`CollectLostTracks()`、`CollectJammingTracks()`
- **谓词查询**：`ContainsExternalTargetId()`、`CountTracksByStatus()`、`CountJammingTracks()`

### RadarInputValidation — 结构化输入校验

- 三级严重度：`kInfo`（语义提示）、`kWarning`（需关注）、`kError`（应阻断）
- 校验项覆盖：时间步合法性、目标字段有限性、位置完整性、ID 唯一性、RCS 非负
- 返回 `std::vector<ValidationIssue>`，每条包含严重度、错误码、目标索引、可读消息
- `HasValidationError()` 辅助判断是否存在阻断级问题
