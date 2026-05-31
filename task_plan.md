# 任务计划：JSBSim 飞行机动模块集成重构

## 目标

将 JSBSim flight_dynamic 集成重构为工程化模块：版本可追溯、XML 合同清晰、初态/配平可验证、控制接口分型明确、机动测试分层可验收。

## 已完成

阶段 0-13 全部完成，核心交付：

- **适配层**：`JsbsimAdapter` 职责收敛，`PropertyNames.h` 集中管理，trim recovery 修复（`InitModel()` 重置 FCS 内部状态）
- **控制接口**：`AircraftControlProfile` 分型（direct surface/native AP/generic AP/FBW），8 机型 profile 快照
- **机动语义**：`kFlyToWaypoint`/`kOrbit`/`kSetHeading`/`kSetAltitude`/`kSetPitch`/`kSetRoll`/`kTakeoff`/`kLand`
- **能量管理**：`UpdateEnergyManagement()` + `UpdatePitchChannel()`，按机型 profile 限幅
- **引擎管理**：`EngineManager` 自动检测活塞/涡喷/涡桨，抬轮速度从翼载实时计算
- **起降控制**：起飞四阶段（引擎→滑跑→抬轮→爬升），着陆 pitch-for-speed/throttle-for-altitude PD
- **测试体系**：五层 CTest（smoke/contract/controllability/performance/known-limit），裸机基线，gtest→CSV 迁移
- **CSV 工具**：`takeoff_land_csv`、`maneuver_sweep_csv`、`analyze_takeoff.py`

## 当前起降状态（最新代码实测）

| 机型 | 起飞 | 巡航 | 着陆 | 问题 |
|------|:---:|:---:|:---:|------|
| c172x | ✅ | ✅ | ✅ | 全任务通过 |
| c310 | ✅ | ✅ | ❌ | 着陆坠毁（256s） |
| f16 | ✅ | ❌ | — | 超音速飞越航路点 |
| f22 | ⚠️ | — | — | 能离地爬升到 357m，50s 后失控 |
| 737 | ✅ | ❌ | — | 起飞完成但不捕获航路点（560 节太快） |
| B17 | ❌ | — | — | 唯一不能起飞的（82节<Vr=91节） |

## 待处理

### 1. 起降稳定性修复

| 问题 | 机型 | 方向 |
|------|------|------|
| 高空失控 | f22 | FBW pitch 通道在爬升后段发散，需研究 f22.xml pitch FBW 链 |
| 着陆坠毁 | c310 | 着陆 PD 增益需要从 CSV 迭代调优 |
| 超速不捕获 | f16/737 | 需要最大速度保护或自适应航路点半径 |
| 推力不足 | B17 | 验证 4 发引擎是否都收到油门 |

### 2. PD 增益调优

起飞爬升率和着陆速度/高度 PD 增益目前是手工设定（`Kp_speed=0.02, Kp_alt=0.003`），需要从 CSV 时间序列数据迭代优化。

### 3. public 示例（遗留项 8.3/12.4）

展示推荐机型（c172x）+ 5 类机动 + source-debug/prebuilt-debug 调试流程。

## 关键决策（已定）

| 决策 | 理由 |
|------|------|
| 机动层只表达语义目标 | 不同 XML 中同名 property 语义可能不同 |
| 控制器按 profile 分型 | direct surface、native AP、generic AP、FBW 不能共用 |
| 长期 vendor third_party/jsbsim | 可追溯 source+data 基线 |
| XML output 禁写 source tree | adapter 设临时 output path，加载后禁用 |
| 飞行性能测试用 CSV 不用 gtest | 时间序列分析 > pass/fail 断言 |
| Release 预设用于测试 | ~6× 快于 debug |
| 默认 CI：fd_ci (smoke+contract+controllability) | release 预设下 ~5s |

## 工具

- `cmake --preset llvm-ninja-release-local` 构建
- `ctest --preset llvm-ninja-release-local -L fd_ci -j 4` 测试
- `examples/flight_dynamic/takeoff_land_csv <model> <out.csv>` 起降时间序列
- `examples/flight_dynamic/maneuver_sweep_csv <model|ALL> <out.csv>` 机动扫描
- `python3 tools/analyze_takeoff.py [--plot] <csv...>` 分析+绘图
