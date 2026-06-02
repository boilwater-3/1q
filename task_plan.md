# 任务计划：起飞控制器修复（CRASH 机型）

## 目标

修复 7 个 CRASH 机型（L410, OV10, F450, B747, MD11, XB-70, T38）的起飞/着陆问题。

## 背景

分支 `refactor/jsbsim-integration`，fd_ci 3/3 绿。10/28 机型全任务通过，F450 跳过（多旋翼）。

## 根因分析（最终确认）

1. **RunIC 地面弹跳**：`SuspendIntegration()` → dt=0 → 地面力不积分 → 首个 Run(dt>0) 弹跳
   - 涡桨（L410, OV10）静态拉力极大 → 初始 vc 20.8 kts → 弹跳 + 无控翻转
   - 涡扇（737, f16）静态拉力低 → 自然阻尼 → 不受影响
2. **reset XML 高度**：每个机型的 reset00.xml 定义了正确 AGL → 加载后 gear 在跑道面
3. **阶跃 elevator**：Vr 时 elevator 0→-0.3 阶跃 → 大推重比机型（B747）产生灾难性滚转
4. **AP lateral detector 误判**：`ap/autopilot-roll-on` 从共享 Autopilot.xml 泄漏 → 误选 `kGenericAutopilotBridge` → 无副翼控制

## 阶段

### 阶段 1 ✅ — 初始速度修复
- velocity=0 for ground start
- 结果：RunIC 后 UVW=0 正确，但首个 Run() 仍弹跳（非速度问题，是力平衡问题）

### 阶段 2 ✅ — 滑跑横侧控制
- StartEngine: roll_mode=0, roll_ap_on=true（wings-level during ground roll）

### 阶段 3 ✅ — 渐进式旋转
- elevator 3s ramp: `el = -0.3 * min(elapsed/3.0, 1.0)`
- 去掉 ConfigureForClimb 中的阶跃 elevator

### 阶段 4 ✅ — F450 跳过
- 标记为 multirotor → skipped

### 阶段 5 ✅ — 地面弹跳修复
- 加载 reset00.xml → 正确 AGL（gear 在跑道面）
- SettleInitialGroundState（HoldDown 5 帧 + 刹车 + idle 油门）
- VehicleStateMapper：altitude=0 时保留 XML 值
- InitRunning throttle reset + 涡桨 prop-advance 设置

### 阶段 6 ✅ — F4N 回归修复
- 根因：reset AGL 1.14m 暴露了 AP kOwnAutopilot 缺少直接 lateral 控制
- 修复：kOwnAutopilot 路径增加 UpdateDirectHeadingLateral 安全网
- 辅助：altitude=0 override（reset 高度导致 nose-only 接地）

### 阶段 7 ✅ — 硬编码清理
- 删除 Tier 1 显式 profile 表（7 个机型硬编码 → 0 个）
- lateral_interface 全动态检测：FBW > OwnAP > GenericAP > DirectSurface
- OV10 guard：单属性泄漏不配 kGenericAutopilotBridge
- indexed_throttle：`engine_count > 1` 动态
- 能量管理：按属性分类（FBW/发数/mixture）替代机型名字符串
- 删除 F4N 硬编码 altitude override

## 🆕 阶段 8 — 剩余 CRASH 修复 — `pending`

### 8.1 阶跃 elevator（B747, MD11, XB-70）
- 现状：3s ramp 对大推重比机型不够
- 方案：增加 ramp 时间或按机型类别分档（重型机更慢）
- 文件：`src/flight_dynamic/guidance/Maneuver.cpp`

### 8.2 涡桨不启动（L410, DHC6）
- 现状：`FGTurboProp` 零 RPM 无法自建转速，`SetPropAdvance` 已加但 RPM 仍 0
- 方案：JSBSim 涡桨模型局限，需研究 propeller spool-up 初始化
- 文件：`src/flight_dynamic/adapter/JsbsimAdapter.cpp`（SetPropellerAdvanceState）

### 8.3 任务不完成（OV10, Concorde, C130, p51d）
- OV10：不再 crash，但巡航目标过高/速度过慢，2500s 飞不到航路点
- Concorde/C130/p51d：地面 stuck 不动
- 方案：巡航高度引擎类型分类调整（OV10 被标为 turbine_engine 而非 turboprop）

### 8.4 着陆 crash（T38, A4, DHC6）
- T38：全程完成起飞+巡航，着陆 pitch=66° 坠毁
- A4：21.5s crash
- DHC6：2.4s crash
- 方案：着陆控制器调参，可能和 epoch 2 中的 F4N 修复相似

## 关键决策

| 决策 | 理由 |
|------|------|
| 全动态检测 | 无机型名硬编码，靠属性存在性判断 |
| reset XML 加载 | 每机型 AGL 正确，非手工维护映射表 |
| 渐进旋转 3s | 模拟飞行员柔和操作 |
| OV10 guard 规则 | `ap/autopilot-roll-on` 单属性泄漏，需额外证据 |
| FBW 优先 | f16 有 ap/heading_hold（FBW 暴露）但需 kFbwRateCommand |

## 工具

- `cmake --preset llvm-ninja-release-local` 构建
- `ctest --preset llvm-ninja-release-local -L fd_ci -j 4`
- `build/llvm-ninja-release-local/bin/takeoff_land_csv <model> <out.csv>`
- `python3 tools/analyze_takeoff.py [--plot] <csv...>`
