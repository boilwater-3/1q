# JSBSim 机型底层架构与参数字典全集

结合前期的宏观结构调研与代码调试中暴露的问题（起降弹跳、FBW 积分发散等），我们明确了：JSBSim 的真正难点和机型差异并不在于顶层 XML 标签的有无，而在于**引擎物理机制、控制闭环程度和地面反力刚度常数**等底层实现。

本报告分为两部分：**第一部分**是对底层机制差异的详细分类解读，**第二部分**是对当前 `third_party/jsbsim/aircraft` 下所有 42 个有效机型的具体参数详尽查表（字典）。它可以直接为你的 `JsbsimAdapter` 和机动状态机编写特殊适配逻辑提供事实依据。

---

## 🔬 第一部分：三大核心机制差异分类

### 1. 发动机物理代差致灾机制
- **活塞 (Piston / `piston_engine`)**：推力响应平缓，高度依赖 `magneto`（磁电机）、`starter` 和 `mixture`（混合比）。典型的地面拉力较弱，例如 c172x 满油门也能用刹车憋住不弹跳。
- **涡喷/涡扇 (Turbine / `turbine_engine`)**：基于 `GetSteadyState` 迭代。其特性是低速（V=0）时拉力极低，必须随速度增加才建立有效拉力。737 / F16 起飞初始帧安全。
- **涡桨 (Turboprop / `turboprop_engine`)**：这是导致你的 **L410** 和 **OV10** 在 `RunIC` 与初始帧直接翻转坠毁的元凶。由于螺旋桨模型在零速时效率达到物理极点，初始化瞬间会产生高达几千甚至上万磅的静态拉力。如果不加入专门的 `HoldDown` settle，必挂无疑。

### 2. 飞控系统 (FCS) 闭环与外部干扰冲突
- **Kinematic (简单运动学映射)**：只使用了 `aerosurface_scale` 和 `summer` 等。它代表这架飞机是“傻瓜型”的，你的 AP 可以直接向 `fcs/elevator-cmd-norm` 写指令，而不需要担心里面的积分器打架（代表：c172, B17）。
- **Scheduled Gain (增益调度)**：加入了 `scheduled_gain` 和 `lag_filter`，随动压或马赫数衰减舵效。
- **FBW (电传飞控 / 高度动态)**：使用了 `fcs_function`, `pid`, `switch` 等。XML 内部通过角速度/G力做了真正的闭环反馈。这就意味着：外部不能简单调用原生的 `DoTrim`，也不能强塞姿态指令，否则会导致积分器爆表发散坠毁（代表：f16, L410）。

### 3. 起落架/支柱地效几何悖论
- JSBSim 用 `spring_coeff` (弹簧刚度) 和 `damping_coeff` (阻尼) 算接触力。
- 刚度跨度从 J3Cub 的 `1,000` lbs/ft，到 B747 的 `150,000` lbs/ft，乃至 787 的 `833,000` lbs/ft。
- 如果某重型机型（如 L410）定义的自然悬垂高度是 AGL 7.1 ft，而仿真初始化将它扔在了 AGL=0，则巨大的弹簧刚度将瞬间产生百万磅的弹力。

---

## 📚 第二部分：JSBSim 现有机型全参数字典

以下数据是通过代码直接从各机型的 XML 节点中深度遍历抽取的。你可以通过它快速确认各个机型的成分：

### A. 引擎装配与动力分布字典

| 机型 | 发动机底层类型 | 数量 | 引擎原型配置文件 |
| :--- | :--- | :--- | :--- |
| **737** | `turbine_engine` | 2 | CFM56 |
| **787-8** | `turbine_engine` | 2 | trent_1000 |
| **A4** | `turbine_engine` | 1 | J52 |
| **B17** | `piston_engine` | 4 | R-1820-97 |
| **B747** | `turbine_engine` | 4 | GE-CF6-80C2-B1F |
| **Boeing314** | `piston_engine` | 4 | WrightGR-2600 |
| **C130** | `turbine_engine` | 4 | t56 |
| **Concorde** | `turbine_engine` | 4 | Olympus593Mrk610 |
| **DHC6** | `turbine_engine` | 2 | PT6A-27 |
| **F4N** | `turbine_engine` | 2 | J79-GE-11A |
| **F80C** | `turbine_engine` | 1 | J33-A-35 |
| **J3Cub** | `unknown` | 1 | Continental A-65-8 |
| **L410** | `turboprop_engine` | 2 | engtm601 |
| **MD11** | `turbine_engine` | 3 | CF6-80C2 |
| **OV10** | `turbine_engine` | 2 | T76 |
| **T37** | `turbine_engine` | 2 | J69-T25 |
| **T38** | `turbine_engine` | 2 | J85-GE-5 |
| **X15** | `rocket_engine` | 1 | XLR99 |
| **XB-70** | `turbine_engine` | 6 | YJ93-GE-3 |
| **c172p / r / x** | `piston_engine` | 1 | eng_io320 / engIO360C |
| **c182** | `piston_engine` | 1 | engIO540AB1A5 |
| **c310** | `piston_engine` | 2 | engIO470D |
| **f15** | `turbine_engine` | 2 | F100-PW-229 |
| **f16** | `turbine_engine` | 1 | F100-PW-229 |
| **global5000**| `turbine_engine` | 2 | BR710 |
| **p51d** | `unknown` | 1 | Packard-V-1650-7 |
| **pc7** | `unknown` | 1 | PT6A |
| **t6texan2** | `turbine_engine` | 1 | PT6A-68 |
| **wrightFlyer1903** | `piston_engine`| 1 | wright1905_engine |
| **x24b** | `rocket_engine` | 1 | XLR99 |

*(注：部分无动力载具如 `F450`, `mk82`, `sgs233`, `weather-balloon`, `Shuttle` 等未列出，它们没有 `<propulsion>` 或无 engine 节点)*

### B. 飞控通道 (FCS) 架构复杂度字典

| 机型 | 飞控通道数 | 飞控包含的关键运算组件 | 适配层兼容性评估 |
| :--- | :--- | :--- | :--- |
| **737** | 7 | kinematic, aerosurface_scale, scheduled_gain, summer | **中** (含随速衰减，不排斥外部指令) |
| **787-8** | 6 | kinematic, aerosurface_scale, scheduled_gain, summer | **中** (同上) |
| **B747** | 1 | kinematic, aerosurface_scale, scheduled_gain, summer | **中** |
| **Concorde** | 4 | aerosurface_scale, scheduled_gain, kinematic, actuator, summer | **中** |
| **C130** | 1 | kinematic, aerosurface_scale, summer | **简单** |
| **c172x** | 4 | aerosurface_scale, kinematic, pure_gain, actuator, summer | **简单** (直写指令即可) |
| **B17, c182, c172p, c172r, p51d** | 4 - 6 | kinematic, aerosurface_scale, summer | **简单** (全裸露通道) |
| **L410** | 9 | aerosurface_scale, scheduled_gain, kinematic, pure_gain, **switch**, summer | **高** (存在复杂动态判断节点，接管较难) |
| **c310** | 6 | aerosurface_scale, kinematic, **fcs_function**, pure_gain, summer | **高** (闭环查表或内建积分) |
| **f16** | 10 | aerosurface_scale, **pid**, scheduled_gain, kinematic, **fcs_function**, pure_gain, **switch**, summer | **极高** (完整 FBW 闭环，需特殊桥接模式) |
| **pc7** | 1 | **switch** | **高** |
| **weather-balloon** | 3 | pure_gain, **switch**, **fcs_function** | **高** (内部自治逻辑) |
| **DHC6, F4N, J3Cub, mk82**| 0 | (无) | **无** (纯几何气动，无控制面映射) |

### C. 支柱地效几何刚度字典 (起降关键参数)

这是你用以分析起落架“穿透”还是“反弹”的直接数据参考基准，所有数据均摘自机型的 `<ground_reactions>`：

| 机型 | 触地点 (Contact) | 弹簧刚度 (spring_coeff) | 阻尼系数 (damping_coeff) |
| :--- | :--- | :--- | :--- |
| **c172x** (标杆) | Nose Gear | `1800.0` | 500.0 |
| | Left Main Gear | `5400.0` | 160.0 |
| **J3Cub** (最软) | LEFT/RIGHT_MAIN | `1000.0` | 500.0 |
| **L410** (待修复) | NOSE_LG | `19756.8` | 1411.2 |
| | LEFT/RIGHT_MLG | `70560.0` | 2822.4 |
| **OV10** (待修复) | NOSE_LG | `20160.0` | 1440.0 |
| | LEFT/RIGHT_MLG | `72000.0` | 2880.0 |
| **f16** | LEFT/RIGHT_MLG | `37500.0` | 7500.0 |
| **737** | Nose Gear | `90000.0` | 4000.0 |
| | L/R Main Gear | `120000.0` | 10000.0 |
| **B747** | LEFT/RIGHT_MLG | `150000.0` | 174605.0 |
| **Concorde** | LEFT/RIGHT_FWD | `408000.0` | 81600.0 |
| **XB-70** | LEFT/RIGHT_MLG | `500000.0` | 100000.0 |
| **787-8** (最硬) | LEFT/RIGHT_MAIN | `833000.0` | 166600.0 |

*(对比发现，重型飞机的刚度呈指数级攀升。这在 RunIC 的 SuspendIntegration 被取消或者起落架几何高度设置不当的时候，轻微的高度偏移就会放大成几十万磅的弹簧反弹力。)*
