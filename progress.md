# 进度：JSBSim 飞行机动模块

## 当前状态

分支 `refactor/jsbsim-integration`。16 可用机型全任务通过。阶段 12 完成，阶段 13 待开始。

## 全机型 takeoff_land_csv 测试结果（2026-06-04）

### ✅ 完成（16 机）
| 机型 | 时间 | 备注 |
|------|------|------|
| c172x | 1313s | |
| c172p | 840s | |
| c172r | 1037s | |
| c182 | 793s | |
| c310 | 406s | |
| 737 | 1136s | |
| **MD11** | **2436s** | ✅ 阶段 12 修复（flare 解耦） |
| f16 | 179s | |
| f15 | 242s | |
| A4 | 359s | |
| F4N | 244s | |
| T38 | 338s | |
| DHC6 | 2381s | |
| OV10 | 1420s | |
| F80C | 359s | |
| Boeing314 | 721s | |

### ⚠️ 已知问题
| 机型 | 状态 | 根因 |
|------|------|------|
| **B747** | ✅ 1485s | ✅ 13b 连续化 cruise_factor 修复 |
| **XB-70** | 💥 crash | JSBSim delta wing 模型俯仰不稳定，不修 |
| Concorde | ⏰ timeout | 引擎燃油兼容性（阶段 8.2） |
| C130 | ⏰ timeout | 螺旋桨缺 gearratio（阶段 8.2） |
| L410 | ⏰ timeout | 引擎 cutoff-cmd（阶段 8.2） |

## 已完成阶段

| 阶段 | 内容 | 关键 commit |
|------|------|------------|
| 1-7 | 初始修复 | — |
| 8.1 | 重型机起飞稳定化 | 06b37c3f |
| 8.3a | MD11 fly-to/landing | 5d5f7ac5 |
| 8.3b | 非指令升空+CLmax+DHC6 | 55c935f8 |
| 8.3c | OV10 重量分类巡航高度 | 2efd8cb9 |
| 9a | 航路点速度+速度包线 | cd8900e9 |
| 9b | F80C+B747 flare | 8484c90c |
| 10 | XML 配置驱动进近重构 | bd2a4f59 |
| 11 | XML 配置契约 | — |
| 12a | landing/flare capability 迁移 | — |
| 12b | landing 控制律常量命名化 | — |
| 12c | EngineManager 参数边界确认 | — |
| 12d | 示例程序策略外移 | — |
| 12e | 物理包线+升限+MD11修复 | 97c05b45 |

## 阶段 13 计划

| 子阶段 | 描述 | 风险 | 状态 |
|--------|------|------|------|
| 13a | 获取推重比（运行时估算） | 中 | ✅ 完成 |
| 13b | 翼载连续化速度包线 | 中 | ✅ 完成 |
| 13c | log10(Iyy)→连续化 rotation | 低 |
| 13d | wing_loading→spd_priority; V_stall→approach fallback | 低 |
| 13e | CLmax/climb_pitch/Vr_factor XML override | 低 |
| 13f | B747 着陆回归修复 | 独立 |

执行顺序：13a→13c→13d（第一批）→ 13b（第二批）→ 13e→13f

## git 历史

```
97c05b45 feat: physics-based speed envelope, ceiling clamp, scenario config
bd2a4f59 Refactor landing guidance profile configuration
8484c90c fix: F80C fly-to timeout and B747 flare improvements
cd8900e9 feat: waypoint speed field and speed envelope management
2efd8cb9 fix: OV10 TIMEOUT — weight-based cruise altitude for turbine aircraft
55c935f8 fix: uncommanded liftoff, CLmax, DHC6 landing
5d5f7ac5 fix: MD11 fly-to/landing crash — four root causes
06b37c3f fix: stabilize heavy aircraft takeout (B747, MD11)
```
