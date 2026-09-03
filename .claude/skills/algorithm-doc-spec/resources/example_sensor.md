---
Status: active
Last-reviewed: 2026-08-30
Authority: AR 算法登记与实现边界
Answers: AR 用了哪些算法/部件、各自实现到什么地步、边界在哪、哪些刻意不实现
---

# Airborne Radar 算法登记

本文是 Airborne Radar (AR) 算法与部件清单及边界的权威。算法本身的逐步逻辑读代码（`src/airborne_radar/`）；本文只回答“用没用/到哪步/为什么不做”。模块级边界（dt_sec、环境/RF 事实、输出/失败语义、滤波后端选型原则、public API 边界）见 [boundaries.md](boundaries.md)。

## 算法登记表

| 算法/部件 | 意图 / 核心转换 | 实现状态 | 证据与单测 |
|---|---|---|---|
| 扫描和波束控制 | 解析扫描中心、坐标组合、天线波束宽度与方向增益；TWS/TAS 模式逐周期推进波位 | session-wired | [evidence: tests/unit/airborne_radar/ar_signal_scan_schedule_test.cpp] |
| STT 指定航迹跟随指向 | 外部只指定目标，STT 波束指向自动由指定航迹局部位置换算 | session-wired | [evidence: tests/unit/airborne_radar/ar_stt_track_follow_test.cpp] |
| 统一物理探测与 RF 干扰链 | 实际发射 $\to$ echo $\to$ incident RF $\to$ 前端账本 $\to$ 检测单元 $\to$ CFAR 判决 | session-wired | [evidence: tests/unit/airborne_radar/ar_signal_pipeline_test.cpp] |

实现状态说明：
- **session-wired**：已接入 `ArSession` / `SignalPipeline` 主链路，覆盖配置、执行、重放与集成。
- **characterized**：具备局部测试与确定性质量矩阵，尚未接入主链。
- **experimental**：可编译且有单元测试，尚未完成全场景标定。
- **internal/受控**：库内私有能力，不暴露为公开契约。

---

## 核心算法详述

## 1. 扫描调度与天线波束控制 (Scan Schedule & Beam Pointing Resolution)

### 1.1 算法意图与调用时机
- **业务意图**：将任务配置中的扫描范围、波位中心与周期步进转换为当前时刻实际天线 boresight 指向，并根据天线物理孔径与波长计算等效波束宽度与单程增益。
- **调用时机与宿主**：由 `ArSession` 在 `PrepareCycle` 阶段调用 `ScanScheduleResolver`，生成的指向同时驱动发射合成、接收增益与检测单元。
- **公共/私有归属**：波位序列构建与步长解析委托公共单源 `src/common/radar/ScanScheduleRuntime.h`，AR 模块仅保留模式语义接线。

### 1.2 数学与物理模型
- **输入与坐标系**：
  - 输入：扫描中心 $(\theta_c, \phi_c)$ (deg)，载频 $f$ (Hz)，天线物理孔径尺寸 $L_a, W_a$ (m)，平台姿态角与安装角偏置。
  - 参考系转换：目标位置首先投影至雷达参考系，天线体坐标通过旋转合成转换为 ECEF 视轴指向。
- **核心公式**：
  1. **波束宽度解析**（按轴独立解析，优先级：显式命令 > 标称值 > 孔径物理推导）：
     $$\theta_{3\text{dB}} \approx \frac{k_\lambda \cdot c}{f \cdot D_a}$$
     其中 $c$ 为光速，$D_a \in \{L_a, W_a\}$ 为对应轴物理孔径，$k_\lambda$ 为天线照射因子（缺省 0.886）。
  2. **扫描序列生成**：
     波位序列按模式步长展开，当前周期波位索引为 $i = k_{\text{cycle}} \pmod N$。
     在 TAS 模式下，网格采样步长减半；若启用 `prefer_dense_tas_sampling` 则进一步减半。
  3. **方向增益计算**（高斯波束近似）：
     $$G(\theta, \phi) = G_0 \cdot \exp\left( -2.776 \cdot \left[ \left(\frac{\Delta\theta}{\theta_{az}}\right)^2 + \left(\frac{\Delta\phi}{\phi_{el}}\right)^2 \right] \right)$$

### 1.3 实现边界与工程简化
- **复杂度与开销**：$\mathcal{O}(1)$ 确定性计算，单轴采样上限截断为 4096 点，严禁浮点累加漂移。
- **物理简化与假设**：天线增益采用主瓣高斯展开近似，旁瓣采用统一底噪电平模型；非相干脉冲积累增益采用 $10\log_{10}(N)$ 经验近似。
- **姿态稳定解耦**：机体稳定模式直接使用挂架相对角；惯性/对地稳定模式先用平台当前姿态反解天线挂架指向，确保波束不随机体晃动漂移。

### 1.4 边界保护与降级策略
- **孔径参数退化**：三级波束宽度解析均无有效值时返回 0（不抛异常、不中断周期），调用方必须保证 commanded 或 nominal 至少其一有效。
- **限位越界保护**：扫描范围由机械限位与电扫限位取交集；若扫描中心落在限位外，自动回退至合法视场中心。

### 1.5 反直觉点与工程陷阱
> [!IMPORTANT]
> - **旁瓣对消与自适应波束只作用于接收态**：公开发布的 `emission_frame` 只承载天线发射特征，ECCM 的方向图陷波零陷仅在接收机内部计算，不得混入外部发布的发射辐射帧。
> - **孔径属于可重放硬件状态**：波长由实时载频动态计算，但天线物理孔径 $L_a, W_a$ 属于静态硬件，必须在 Session Replay 快照中原样保留。

### 1.6 证据链
- **源码入口**：`src/airborne_radar/dwell/ScanScheduleResolver.cpp`
- **单测覆盖**：[evidence: tests/unit/airborne_radar/ar_signal_scan_schedule_test.cpp]
- **集成覆盖**：[evidence: tests/unit/airborne_radar/ar_beamwidth_resolution_test.cpp]

---

## 非目标（刻意不实现的算法）

1. **阵面逐元相控阵电磁全波仿真**：不建模 T/R 组件的微观电磁互耦，采用解析天线阵方向图与波束展宽模型替代。
2. **非线性大失真功放建模**：发射链路采用线性功率包络，不实现饱和谐波失真仿真。
