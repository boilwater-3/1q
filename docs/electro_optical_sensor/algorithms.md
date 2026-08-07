---
Status: active
Last-reviewed: 2026-08-07
Authority: EOS 算法登记与实现边界
Answers: EOS 用了哪些算法、各自实现到什么地步、边界在哪、哪些刻意不实现
---

# EOS 算法登记

本文是 EOS 算法清单与边界的权威。算法本身的逐步逻辑读代码（`src/electro_optical_sensor/foundation`、
`src/electro_optical_sensor/pipeline`）；本文只回答"用没用/到哪步/为什么不做"。模块级边界（dt_sec、
帧级 config 语义、public API 边界）见 [boundaries.md](boundaries.md)。

## 算法登记表

| 算法/部件 | 意图（一句话） | 状态 | 证据 |
|---|---|---|---|
| 配置到内部执行映射 | 四域配置变成 pipeline 可执行参数 | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_session_composition_root_test] |
| runtime patch 原子解析 | 校验运行期变更、拒绝无效 patch、按需重置扫描相位 | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_runtime_config_resolver_test] |
| 环境因子解析 | 场景/大气观测映射为 aerosol、turbulence、radiance bias、分子密度因子 | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_environment_model_test] |
| 辐射传输 | 路径长度/云量/分子密度/气溶胶/湍流/模型类型 → 透过率与路径辐射惩罚 | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_radiative_transfer_test] |
| 光学几何 | 孔径面积、视场立体角、衍射、GSD | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_foundation_test] |
| 红外辐射 | Planck 辐射、发射率、背景辐射、路径透过率 → IR SNR | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_pipeline_test] |
| 可见光辐射 | 太阳辐照、反射率、投影面积、路径影响 → visible SNR | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_pipeline_test] |
| 背景噪声与 NEP | 背景噪声统计、抑制权重、等效噪声、有效信号功率 | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_noise_model_test] |
| 空间可分辨性 | 目标尺度/GSD/MTF/采样效率 → 成像质量增益 | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_spatial_spectrum_test] |
| 杂散光过滤 | 太阳-目标夹角/云量/遮光罩 → 近太阳干扰抑制 | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_straylight_test] |
| 扫描/FOV/范围门控 | 推进扫描相位；FOV 决定记录成员，范围只决定检测资格 | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_pipeline_test] |
| 通道融合与结果组装 | 合成 IR/visible/fused SNR，由 controller 组装 raw output 与 result | session-wired | [evidence: tests/unit/electro_optical_sensor/eos_cycle_output_builder_test] |
| 外部输出坐标转换 | caller-side helper；外部坐标检测转 EOS 输出，不承担 controller 结果组装 | session-wired | [evidence: tests/integration/electro_optical_sensor/eos_session_test] |

状态取值：所有 EOS 算法当前均为 **session-wired**——已接入 `EosPipeline`/`EosController`，覆盖 config、
输出/abort、replay 与 session 集成。EOS 当前没有 characterized/experimental 候选算法；能力晋级必须逐级
提供证据，不得跳级。

## 配置、环境 preset 与运行期映射

- **意图**：`MapSessionToInternal` 把四域配置变成 pipeline 可执行参数；`BuildModelConfigFromScenario`
  在内部从 preset 派生辐射算法、气溶胶系数和湍流系数。
- **实现边界**：
  1. replay 的 session-config payload 只记录 `preset + atmospheric_physics`，与公开 source of truth
     完全一致；内部派生算法和数值不进入 schema，也不形成第二套可配置状态。
  2. 调用方不选择具体辐射算法，也不填写 custom 因子。`enable_physical_model=false` 时观测值不参与
     物理修正；为 true 时气压和温度必须有限且大于零、相对湿度必须位于 `[0,1]`。
  3. 环境解析固定为：preset 建立内部基线 → 每周期高度、云量和风速自动动态修正 → 启用的标准大气
     物理观测追加修正。其中湿度调气溶胶因子、温度调湍流因子，气压与温度按理想气体比例派生空气
     分子密度因子，并只缩放基线分子衰减。
- **反直觉点**：旧"Mission Profile 跨域覆写 `policy.detection.minimum_snr_db`"语义已消除。配置不再
  有隐式优先级，任何字段的赋值即最终决定（档位在前、微调在后时微调胜出）。
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_session_composition_root_test]
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_environment_model_test]
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_session_config_builder_test]
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_pipeline_test]

## 目标级上下文：路径、大气、空间分辨、杂散光

- **意图**：每个目标构造 `DetectionComputationContext`，在帧级上下文基础上补充目标相关量
  （路径传输、范围门控、GSD、空间可分辨性、杂散光抑制）。
- **实现边界**：
  1. `imaging_quality_gain` 是几何质量和空间频谱质量的组合结果，并继续影响背景噪声场景复杂度——
     空间分辨不是输出装饰项，而会反馈到 SNR 判定。
  2. 范围门控只决定最终检测资格；视场内但超出 `dmin_m/dmax_m` 的目标仍计算通道 SNR 并保留记录
     （见 boundaries.md 帧级 config 语义）。
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_pipeline_test]
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_spatial_spectrum_test]
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_straylight_test]

## 红外通道

- **意图**：基于温差辐射与探测器噪声之间的关系估计 IR SNR（Planck 辐射 → 谱辐射差 → 带宽积分 →
  红外对比 → 接收功率 → NEP/噪声归一）。
- **实现边界**：
  1. 该通道按物理链路处理温度、发射率、背景辐射、路径透过率、孔径、距离和积分时间；具体逐步逻辑
     读代码，不在文档中重述。
  2. SNR 方向性约束：目标温度升高、带宽增大（固定中心波长）、距离减小、大气状况改善，分别应使
     IR SNR 不降低。
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_foundation_test]
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_pipeline_test]

## 可见光通道

- **意图**：基于照明、反射率、目标投影面积、路径透过率和光子噪声估计 visible SNR。
- **实现边界**：
  1. 该通道不把红外温差当作主要信号源。白天可见光权重更高，夜间 fused 权重应向红外倾斜。
  2. 目标投影面积在通道链路中只应用一次，避免重复放大目标。
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_foundation_test]
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_pipeline_test]

## 噪声、NEP、空间频谱与杂散光

> 本节故意极简：详细参数清单（背景噪声 sigma、抑制权重、NEP 各分量、GSD/MTF 系数、太阳夹角阈值）
> 是 textbook 物理，读代码，不在文档中复述。本节只保留三条不可从代码推断的高价值边界。

- **意图（不可从代码推断）**：EOS 的检测判定**不是**简单的"信号大于阈值"。噪声和成像质量由背景
  噪声、背景抑制、NEP、空间频谱、杂散光等多项内部模型共同形成，组合后才决定 SNR 与门限。
- **实现边界（不可从代码推断）**：这些算法位于 **foundation 层，是基础物理实现，不是 public
  customization surface**。外部用户只能通过硬件、任务、策略、环境和输入影响这些模型，不能直接替换
  或自定义 foundation 算法类型。
- **反直觉点（`_cm2` 单位后缀契约）**：`detector_area_cm2` 的厘米制单位与
  `detector_detectivity_cm_sqrt_hz_per_w` 的 D* 量纲绑定，二者直接进入 NEP 计算。公开名称必须保留
  `_cm2` 后缀——这是契约，不是命名风格：避免调用方误把其他面积单位（m²、mm²）的数值直接复制进来
  造成数量级错误。
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_foundation_test]
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_noise_model_test]
- **证据**：[evidence: tests/contract/check_cross_domain_naming]

## 融合、检测记录与仿真归属

- **意图**：pipeline 先得到红外 SNR 和可见光 SNR，再依据工作模式（`kInfraredOnly`/`kVisibleOnly`/
  `kFused`）生成最终检测判定。
- **实现边界**：
  1. raw detection 只保留真实传感器侧字段（detection id、方位/俯仰、距离、SNR、通道）；输入目标
     ID/name 等仿真便利字段不得进入 raw output。
  2. 仿真归属由 `EosCycleResult` 和 debug view 承接；debug view 把 raw output 和输入目标合并展示，
     服务开发排查；lifecycle recorder 跟踪 found/lost/optional not-detected 跨周期事件；replay 保存
     cycle input/output/result 和 failure marker。
  3. `EosCycleOutputAdapter` 是 caller-side helper，将外部坐标检测转换为 EOS 输出，不承担 controller
     结果组装职责。
- **证据**：[evidence: tests/unit/electro_optical_sensor/eos_cycle_output_builder_test]
- **证据**：[evidence: tests/contract/electro_optical_sensor/eos_public_api_convenience_test]
- **证据**：[evidence: tests/replay/electro_optical_sensor/eos_replay_session_test]

## 非目标（刻意不实现的扩展）

1. **public pipeline/controller/environment/foundation 自定义点**：当前没有用户替换 controller、
   pipeline 或环境模型的 public API。需要改变物理行为应通过配置和输入表达。
2. **foundation 算法升级为 public contract**：foundation 是内部可测试实现。若要从 internal 变成
   public API，必须在本文 `[evidence: ...]` 标注中记录扩展理由和兼容策略，当前不扩展。
3. **环境 preset 简化为无语义 flat 参数**：preset 到物理参数的映射是设计内容，不暴露为 flat 参数，
   也不允许调用方填写 custom 气溶胶/湍流因子。
4. **仿真真值混入 raw output**：不把仿真目标 ID/name、debug view、lifecycle 或 replay 当作真实传感器
   输出。需要排查仿真归属应消费 `EosCycleResult`、debug view、lifecycle 或 replay。
5. **Mission Profile 跨域隐式覆写**：不恢复旧"Mission Profile 覆写 policy"的隐式优先级语义
   （见配置映射反直觉点）。
