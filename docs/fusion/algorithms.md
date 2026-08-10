---
Status: active
Last-reviewed: 2026-08-10
Authority: fusion 算法清单与实现边界
Answers: 每个融合算法怎么实现、边界在哪、反直觉点是什么
---

# Fusion 算法

## 算法清单

| 算法 | 输入 | 输出 | 证据 |
|---|---|---|---|
| 关联分层（键直挂 / 位置 / 方位 / 特征） | 探测记录 + 既有航迹 | 探测 → 航迹归属 | `src/fusion/FusionEngine.cpp`；`tests/unit/fusion/fusion_association_test.cpp` |
| 置信度滑窗融合 | 滑窗内量测 | 每航迹融合置信度 | 同上；`tests/unit/fusion/fusion_confidence_test.cpp` |
| 传感器输出 → 泛型探测记录（官方适配器） | AR 轨迹帧 / ESR 假设 / EOS 探测 / SBIRS 探测 | `DetectionRecord` 列表 | `src/fusion/SensorAdapters.cpp`；`tests/unit/fusion/sensor_adapters_test.cpp` |

## 1. 关联分层

每周期 `Update` 处理顺序：

1. **身份键直挂**：探测按键（≠ 0）分组，同键归并到同航迹（跨源一致性由调用方
   保证）；无对应航迹时创建。
2. **位置关联**：对带位置锚点的航迹建 nanoflann KD-tree（ECEF 三维，double，
   `L2_Simple_Adaptor`，先例复制自 ESR `KdTreeClusterer` 的 adaptor/radiusSearch
   模式，含 1.3.x / 1.5+ 返回类型兼容垫片），无身份探测做半径搜索
   （`position_radius_m`，平方半径语义）。
3. **方位关联**：仅方位探测与带方位锚点的航迹做 `AreBearingsCoherent`
   （`bearing_beamwidth_deg`，az 按最短圆差、el 线性比较）。
4. **特征门限**：`feature_threshold > 0` 且双方都有特征且维度一致时，要求欧氏
   距离 ≤ 门限；否则不构成约束。
5. 候选择优：位置取最近、方位取最短圆差、纯特征取首个通过者；
   未关联 → 新建航迹（合成键）。

### 实现边界

- 每周期成本 **O(N log N + M)**：KD-tree 建树 O(T log T) + 每次半径搜索 O(log T)，
  无全量两两比对。
- **同周期新建航迹不参与本周期后续关联**（自下周期起成为候选）；
  **无身份探测之间本周期互不合并**（合并依赖身份键或后续周期航迹关联）——
  两源对同一新目标的首见周期会各自成航迹，属已知限制（业务层可用身份键规避）。
- 锚点为航迹最近一次量测；KD-tree 使用周期开始时的锚点快照。

## 2. 置信度滑窗融合

- 公式（冻结）：**confidence = Σ 判决值 × 质量归一化 × 权重**，对滑窗内全部
  量测精确求和。
- 权重按 `source_id` 索引 `FusionConfig::source_weights`，缺失按 1.0。
- 每源每航迹 `std::deque` 滑窗（`window_size`），超出驱逐并重算置信度。
- 失跟：航迹每周期未收到量测则 `missed_cycles` +1，超过 `max_missed_cycles` 删除。

### 反直觉点

- **置信度不归一化**：随窗口内证据（探测次数/源数）单调累积，可超过 1.0——
  这是冻结公式的字面语义（更多证据 → 更高置信），不是归一化概率。
- 滑窗按**量测条数**而非周期数截断；同一源高频率探测会更快驱逐旧样本。
- 输出按航迹键升序（`std::map` 迭代序），确定性可复现。

## 3. 传感器输出 → 泛型探测记录（官方适配器）

`fusion/SensorAdapters.h` 为四传感器输出提供默认映射（`Adapt*ToDetectionRecords`，
跳过规则与质量归一化基准如下），集成方即开即用；业务层可自行适配（不修改
本文件）。源通道常量 `kArSourceId=1..kSbirsSourceId=4` 与
`FusionConfig::source_weights` 索引对齐（索引 0 未用）。

| 适配函数 | 跳过规则 | 身份键 | 质量归一化基准 |
|---|---|---|---|
| `AdaptArTracksToDetectionRecords` | `kLost` 轨迹 | `association_key` | `target_probability`，无识别置信度时按状态（`kConfirmed`→1.0 / 其余→0.5） |
| `AdaptEsrHypothesesToDetectionRecords` | `hypothesis_id == 0` | `hypothesis_id` | `confidence` |
| `AdaptEosDetectionsToDetectionRecords` | `detected == false` | 0（方位相干） | `fused_snr_db / 10`（10 dB → 1.0）夹取 [0,1] |
| `AdaptSbirsDetectionsToDetectionRecords` | `detected == false` | 0（方位相干） | `infrared_snr_linear / 4`（WFOV 门限 4.0 → 1.0）夹取 [0,1] |

### 实现边界

- AR 位置 ECEF→LLA（`TryEcefToLla`），转换失败退化为仅身份键记录（`has_position=false`）。
- ESR 射频特征归一化到可比尺度（GHz/MHz/ms/µs），供特征门限启用。
- 归一化基准固化为库默认（示例场景验证过的业务决策），不做配置参数（YAGNI）。
  [evidence: tests/unit/fusion/sensor_adapters_test.cpp 14 用例全分支覆盖]

## 输出语义

`FusedTarget`：库内键、各源通道状态（量测数、最近判决/质量/位置/方位）、
融合置信度、最近更新周期。报告节奏（配置化周期 + 事件触发）属 example 业务层，
引擎不感知周期语义，`cycle` 仅用于失跟统计与时间戳。
