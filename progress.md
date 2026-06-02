# 进度：JSBSim 飞行机动模块

## 当前状态

分支 `refactor/jsbsim-integration`，fd_ci 3/3 绿。commit `ec1c4449`。

**阶段 1-7 完成，阶段 8 待开始。**

## 全机型测试结果（2026-06-02，终版）

### ✅ PASS（10 机）
c172x, c172p, c310, 737, f16, f15, F4N, F80C, Boeing314, pc7

### 🟡 SURVIVE / NO-CRASH（1 机）
- OV10 (2500s no crash): 起飞+巡航正常，航路点未完成（巡航目标 8000m 太高）

### 💥 CRASH（5 机）
- L410 (1070s): 涡桨 RPM=0 无法自启动 → 最终穿地
- B747 (51s): 阶跃 elevator 余震 → roll departure
- MD11 (48s): 同上
- XB-70 (45s): 同上
- T38 (2074s): 着陆 pitch=66° 坠毁

### 🛑 GROUND（3 机）
C130, Concorde, p51d — 停在地面不动

### 💥 QUICK CRASH（2 机）
- DHC6 (2.4s): 涡桨不启动
- A4 (21.5s): 控制问题

### ⏭️ SKIP（2 机）
B17 (Vr 不可达), F450 (multirotor)

## 已完成的阶段

### 阶段 1：初始速度 → velocity=0 ✓
### 阶段 2：滑跑横侧 → wings-level during ground roll ✓
### 阶段 3：渐进旋转 → 3s elevator ramp ✓
### 阶段 4：F450 skip ✓
### 阶段 5：地面弹跳修复 ✓
- reset00.xml 加载 → 正确 AGL
- SettleInitialGroundState (HoldDown 5帧)
- VehicleStateMapper altitude=0 skip
- 涡桨 prop-advance 初始化
### 阶段 6：F4N 回归修复 ✓
- kOwnAutopilot + UpdateDirectHeadingLateral 安全网
### 阶段 7：硬编码清理 ✓
- 删除 7 个显式 profile 表项
- 全动态检测：FBW > OwnAP > GenericAP > DirectSurface
- OV10 guard 规则
- indexed_throttle 动态
- 能量管理按属性分类

## 待处理（阶段 8）

| 子任务 | 机型 | 优先级 |
|--------|------|--------|
| 8.1 阶跃 elevator 调参 | B747, MD11, XB-70 | 高 |
| 8.2 涡桨自启动 | L410, DHC6 | 中 |
| 8.3 巡航/任务完成 | OV10, Concorde, C130, p51d | 中 |
| 8.4 着陆 crash | T38, A4 | 低 |

## git 历史

```
ec1c4449 refactor: remove model-name hardcoding from autopilot profile detection
```
