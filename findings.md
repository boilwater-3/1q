# 发现：JSBSim 飞行机动模块

## 引擎类型与属性

6 类引擎，按 `propulsion/magneto_cmd` 存在与否区分活塞/非活塞：

| 类型 | JSBSim 类 | magneto | starter | mixture | 机型 |
|------|----------|:---:|:---:|:---:|------|
| 活塞 | FGPiston | ✅ | ✅ | ✅ | c172x, c310, B17, J3Cub, p51d |
| 涡喷 | FGTurbine | ❌ | ❌ | ❌ | f16, f22, f15, 737, 747, Concorde |
| 涡桨 | FGTurboprop | ❌ | ✅ | ❌ | L410, DHC6, PC7 |
| 火箭 | FGRocket | ❌ | ❌ | ❌ | X15, x24b |
| 电动 | FGElectric | ❌ | ❌ | ❌ | 无飞机引用 |

抬轮速度公式：`Vr = 1.1~1.2 × sqrt(2×Weight / (ρ × WingArea × CLmax))`，用 `metrics/Sw-sqft` + `inertia/weight-lbs` + `atmosphere/rho-slugs_ft3` 实时计算。

## JSBSim 内部机制

- **FCS 组件内部状态**：`FGFCSComponent::Output` 成员变量不通过 property tree 暴露。`SetProperty()` 只改 SGPropertyNode，不影响 integrator accumulator。必须用 `FGFCS::InitModel()` → `FGFCSChannel::Reset()` → `ResetPastStates()` 重置。
- **DoTrim(0) 对 FBW 机型必然失败**：JSBSim trim solver 无法处理闭环 FBW 状态。f22/c310 等机型每次 trim 都抛异常。
- **刹车模型有限**：JSBSim 地面接触中，满油门 + 轻型飞机（c172x）时刹车几乎无效，引擎启动阶段飞机会弹跳。
- **Debug 系统**：只有位掩码 `debug_lvl`（1=启动消息/2=实例化/4=Run入口/8=周期状态/16=越界检查），无结构化日志。

## 控制架构

```
1Q C++ Autopilot (机动语义层)
  → 写入 ap/* hold 标志 + fcs/elevator-cmd-norm
  → JSBSim 原生 AP/FCS (XML 控制链)
    → PID/积分器/滤波器/LQR
    → 舵面执行器 → 气动模型
```

- `kOwnAutopilot`（c172x, c310）：AP 更新只写 hold 标志，原生 XML AP 执行控制
- `kFbwRateCommand`（f16, f22）：直接写 `fcs/aileron-cmd-norm`；pitch 通过 `fcs/elevator-cmd-norm` 但不被 FBW 充分响应
- `kGenericAutopilotBridge`（737, Concorde）：写 `ap/*` 属性到 guidance 层

## 起降问题根因

| 问题 | 根因 | 数据来源 |
|------|------|---------|
| c310 着陆坠毁 | 着陆 PD 增益未调优，高度/speed 振荡 | CSV: H=323→233→285m 反复 |
| f22 高空失控 | FBW pitch 链在爬升后段发散，50s 坠毁 | CSV: Hmax=357m, 14s 离地 |
| f16/737 超速 | 无最大速度保护，f16=834节，737=560节 | CSV |
| B17 推力不足 | 4×活塞 82 节 < Vr 91 节。需验证 per-engine 油门 | CSV |
| 引擎启动弹跳 | JSBSim 满油门+刹车不能阻止轻型飞机离地 | 所有 c172x 起飞 CSV |

## 调试方法

- **gtest**：仅用于合同/profile/smoke/鲁棒性测试（<10s per test）
- **CSV 示例程序**：飞行性能分析——`takeoff_land_csv`（时间序列）、`maneuver_sweep_csv`（机动扫描）
- **Python 分析**：`analyze_takeoff.py` 读 CSV，输出阶段时间线、速度/高度统计、matplotlib 图表
- **Release 预设**：JSBSim 仿真 ~6× 快于 debug（fd_ci ~5s vs ~34s）
