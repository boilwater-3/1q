# RF 场景两个枚举的白话说明

> 对象：`include/1q/electromagnetics/RfScene.h` 中的 `RfSceneWaveformKind`（波形类别）与
> `RfScenePolarization`（极化类别）。
> 面向非开发人员，术语先给白话定义。文中代码出处一律用文件 + 函数名定位（行号会漂移，不写）。

这两个枚举本身不参与计算，它们是"填在波形/极化字段里的标签"。真正的作用是让求解器据此走不同的
分支、算出不同的损耗：

- 波形类别决定**某一时刻信号开没开、此刻频率是多少**（`src/common/electromagnetics/RfScene.cpp`
  的 `TryEvaluateRfArrivalActivity`）；
- 极化类别决定**收发两侧能量能不能对上**，算出一笔极化失配损耗（同文件内部函数
  `TryPolarizationLoss`，结果写入 `polarization_mismatch_loss_db` 并直接从链路预算里扣掉）。

---

## 1. RfSceneWaveformKind —— "信号怎么发"

**白话定义**：波形类别描述发射机"说话的方式"——信号在时间和频率轴上长什么样。本模块是统计级
仿真（不生成真实信号采样点），所以它只决定两件事：**某一时刻信号开没开**、**此刻频率是多少**，
然后拿去和接收窗口做时间/频率重叠计算，重叠多少就收到多少功率。

| 枚举值 | 白话含义 | 比喻 | 对应构造函数（同头文件） |
| --- | --- | --- | --- |
| `kContinuous` 连续载波 | 活动期间一直开着，频率固定在中心频率不变 | 一直哼一个不变的音调 | `TryCreateRfContinuousWaveform` |
| `kPulseTrain` 脉冲列 | 按固定节奏发一串短脉冲，中间静默 | 有节奏地拍手，拍一下停一下 | `TryCreateRfPulseTrainWaveform` |
| `kLinearSweep` 线性扫频 | 频率从起点匀速滑到终点，滑完一个周期再从头来 | 警笛声由低滑到高、循环往复 | `TryCreateRfLinearSweepWaveform` |
| `kBandLimitedNoise` 带限噪声 | 在一段频带内"沙沙"的噪声，功率摊在整个带宽上 | 收音机没调到台的电流声 | `TryCreateRfNoiseWaveform` |

### 选不同值的实际区别

见 `TryEvaluateRfArrivalActivity`（`src/common/electromagnetics/RfScene.cpp`）：

1. 选 `kPulseTrain`：求解器二分查找当前时刻落在哪个脉冲里，**只在脉冲宽度内算"有信号"**，脉冲
   间隙算没信号——时间重叠分数会因此变小，接收功率跟着变小。额外用到脉宽、重复间隔、脉冲数、
   抖动这几个字段（`first_pulse_time_s` / `pulse_width_s` / `pulse_repetition_interval_s` /
   `pulse_count` / `pulse_jitter_fraction` / `timing_seed` / `timing_epoch`）。
2. 选 `kLinearSweep`：**频率不再是固定值**，按"当前时刻在扫频周期里走到哪一步"算出瞬时频率
   （用到 `sweep_start_frequency_hz` / `sweep_stop_frequency_hz` / `sweep_period_s`）——接收机
   若只盯着某个窄频段，信号大部分时间会扫过去而错过。
3. 选 `kContinuous` 或 `kBandLimitedNoise`：活动期间始终算"有信号"，频率固定在中心频率。两者在
   活动判定上走同一分支，区别是语义（一个是纯净单音，一个是噪声）。

四种共用同一个结构体 `RfWaveformSchedule`——里面所有字段都在，但**只有和所选类别配套的字段有
意义**，其余为零或被忽略。

---

## 2. RfScenePolarization —— "电场往哪个方向摆"

**白话定义**：电磁波是电场在空间里来回摆动形成的波，极化描述的就是**电场摆动的方向**。它分两侧
配对使用：发射侧填在 `RfSceneEmission.polarization`，接收侧填在 `RfSceneReceiverState.polarization`
（均在 `RfScene.h`）。**两侧摆动方向对不对得上，直接决定接收功率打不打折。**

| 枚举值 | 白话含义 | 比喻 |
| --- | --- | --- |
| `kHorizontal` 水平线极化 | 电场沿水平方向来回直线摆动 | 跳绳上下抖动，只能穿过竖缝的栅栏 |
| `kVertical` 垂直线极化 | 电场沿垂直方向直线摆动，与水平互相垂直 | 同上，缝的方向转 90° |
| `kRightHandCircular` 右旋圆极化 | 电场方向不停旋转，端点画圆，按右手方向转 | 螺丝钻进木头时的旋转方向 |
| `kLeftHandCircular` 左旋圆极化 | 同上，转向相反 | 螺丝退出来时的转向 |
| `kUnpolarized` 非极化 | 摆动方向杂乱无章、不固定 | 没有固定抖动方向的乱绳 |

### 选不同值的实际区别

见 `TryPolarizationLoss`（`src/common/electromagnetics/RfScene.cpp`；`RfLinkBudget.cpp` 内
v1 版本逻辑相同）：

1. **两侧填同一个值**：损耗 0 dB，能量完全接住。
2. **同为线极化但一横一竖**（H↔V），或**同为圆极化但一左一右**（RHC↔LHC）：损耗取两侧天线的
   交叉极化隔离度（`cross_polarization_isolation_db`，默认 30 dB）——相当于**基本收不到**，
   功率只剩约千分之一。
3. **一边线极化、一边圆极化**，或**任一侧是非极化**：损耗固定 3.01 dB——正好损失一半功率
   （3 dB 规则），保守取值。

场景设计里这是有真实数值后果的：发射机填 V、接收机填 H，30 dB 就没了，本来能探测到的目标会
变得探测不到。

---

## 3. 关于 `enum class ... : std::uint8_t` 这个写法

行尾的 `: std::uint8_t` 是指定底层存储类型为 1 字节无符号整数。好处有两个：

1. `enum class`（强类型枚举）不允许和普通整数混用，写错了编译器直接报错，不会静默通过；
2. 只占 1 字节、紧凑——这些值还会进回放序列化（如 `EsrReplayFlatbufferCodec` 等多处），小体积有利。

---

## 4. 与 RIR 模块 `scene_polarization` 配置的关联

**对象**：`include/1q/remote_identification_radar/config/RirHardwareConfig.h` 的
`RirReceiverConfig::scene_polarization`。

一句话：`RfScenePolarization` 是全仓库统一的"极化选项清单 + 配对扣分规则"，RIR 的
`scene_polarization` 是 RIR 雷达往这张清单里填的空——"我这台雷达用什么极化"（默认水平极化）。
类型就是同一个枚举，RIR 只是声明了一个取值。

### 运行时这一个值流向哪里

填进配置后，这个值在 RIR 内部**一个格子填两处**（单基地雷达收发共用同一副天线，收发极化视为相同）：

1. **自己的接收侧**（`src/remote_identification_radar/dwell/RirReceiverStateBuilder.cpp` 的
   `Build`）：组装 `RfSceneReceiverState` 时填入 `receiver_state.polarization`。该接收状态随后被
   同目录 `RirRfFrontEndResolver.cpp` 的 `TryResolveRirRfFrontEnd` 使用：对场景里每一条别的
   发射调用 `TryEvaluateRfIncidentLink`，算出落到 RIR 接收机输入端的功率，汇总后判断接收机
   饱不饱和，并作为干扰项进入后面的 SINR 计算。
2. **自己的发射侧**（`src/remote_identification_radar/dwell/RirEmissionFactory.cpp` 的
   `TryBuildEmission`）：RIR 自己发射的脉冲列（注册进 RF 场景的 `RfSceneEmission`）带上同一个
   极化。别的模块评估"RIR 的照射对我算不算干扰"时，拿它和自己的极化配对。

### 数值后果

进入 `TryEvaluateRfIncidentLink` 后，就是第 2 节的三条配对规则在起作用：

| 配对情况 | 损耗 |
| --- | --- |
| 别人的发射极化与 RIR 相同 | 0 dB，全量进来 |
| 同类互垂（如别人垂直、RIR 默认水平；或旋向相反） | 30 dB（取 `cross_polarization_isolation_db`，RIR 配置里同一旋钮，同样从接收机配置带进发射天线结构） |
| 一边线极化一边圆极化，或任一边非极化 | 固定 3.01 dB |

举例：场景里的干扰机若是垂直极化、RIR 保持默认水平，30 dB 隔离直接进链路预算——干扰功率只剩
约千分之一，等于极化对不上把干扰挡在门外。

### 完整链路

场景 JSON 字符串 → 配置加载
（`examples/common/config_loaders/remote_identification_radar/config_loader_detail.h`）→
`RirReceiverConfig::scene_polarization` → 收发两处结构体 → `TryEvaluateRfIncidentLink` 的极化
失配扣分 → 接收功率 / SINR。回放序列化（`src/remote_identification_radar/session/`
`RirReplayFlatbufferCodec.cpp`）也存它，保证复盘一致。

### 附注

这不是 RIR 独有的模式：AR 模块的 `ArHardwareConfig.h` 有同名字段、同样的填法（含
`ArEmissionFactory.cpp` / `ArReceiverStateBuilder.cpp` 两条对应路径），属于"各模块硬件配置 →
公共 RF 场景"的统一约定。
