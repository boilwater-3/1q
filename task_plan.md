# 任务计划：飞行控制阶段修复

## 目标

完成全机型起飞→巡航→着陆自动化飞行。修复 CRASH/TIMEOUT 机型。

## 背景

分支 `refactor/jsbsim-integration`，fd_ci 6/6 绿。15/18 可用机型全任务通过。

## 阶段

### 阶段 1–7 ✅ — 初始修复和基线
- 地面弹跳修复、reset XML、横侧控制、旋转渐进、硬编码清理

### 阶段 8 — 剩余 CRASH 修复

#### 8.1 重型机起飞稳定化 ✅ — `commit 06b37c3f`
#### 8.2 引擎/燃油兼容性（Concorde, C130, L410）— `pending`
#### 8.3a：MD11 fly-to/landing ✅ — `commit 5d5f7ac5`
#### 8.3b：非指令升空 + CLmax + DHC6 ✅ — `commit 55c935f8`
#### 8.3c：OV10 重量分类巡航高度 ✅ — `commit 2efd8cb9`
#### 8.4 着陆 crash 自行修复 ✅

### 阶段 9 — 航路点速度与飞行包线 ✅ — `commits cd8900e9 + 8484c90c`

#### 9a：航路点速度 + 速度包线核心 ✅
- `Waypoint.speed_mps` 字段
- `ApplyEnergyDefaults` 6 档分类速度包线
- ExecuteFlyTo/ExecuteOrbit 速度管理
- 能量管理超速保护
- kDecelerate→kApproach AP altitude hold 修复

#### 9b：回归修复 ✅
- **F80C**: fly-past detection + 航路点距离增大 → 1776s completed
- **B747**: 部分改进（bounce recovery, agl<3m touchdown, 进近速度管理）— 仍有进近阶段物理限制

### 阶段 10 — 配置驱动进近重构 ✅

- 10a XML 属性驱动的配置方案（替代硬编码 MOI 阈值）— `completed`
- 10b B747 着陆进近阶段重构（高空减速、盘旋下降、襟翼管理）— `completed`
- 8.2 引擎兼容性（Concorde 燃油 cross-feed、C130 gearratio、L410 cutoff-cmd）

## 工具

- `cmake --preset llvm-ninja-release-local` 构建
- `ctest --preset llvm-ninja-release-local -L fd_ci -j 4`
- `build/llvm-ninja-release-local/bin/takeoff_land_csv <model> <out.csv>`
- `python3 tools/analyze_takeoff.py [--plot] <csv...>`
