---
Status: draft
Date: 2026-08-18
Review-Baseline: `main` @ `b9da39d9`（docs: finalize p0-p5 delivery write-back）
Authority: RIR 双产品架构（识别结论 + 特征量测帧）Stage A 证据矩阵与冻结契约。
  本文承载出口①（特征量测帧）字段冻结、与统一探测记录 feature 通道的对齐裁定、
  TARGET-OQ-4 裁定修订（豁免→双产品）与 contract 规则 2 修订文案冻结；
  Stage B 实施不在本文范围。非规范性记录，若与库实现冲突，以库为准。
---

# RIR 双产品架构：Stage A 证据矩阵与冻结契约

## 0. 结论速览

- 采纳评估结论（2026-08-18 架构评估）：RIR 应输出**双产品**——出口②识别结论（装备
  使命，形态不变）+ 出口①特征量测帧（新增，合法传感器量测产品）。"只输出结论"是
  从 AR 拆分时的过渡形态，不是终态。
- Stage A 判定：F1/F2/F3/F5/F6/F7 **pass**、F4 **narrow**（fusion 特征门对无效维掩码
  的 mask-aware 升级登记为后续冻结项；11 维固定布局 + 0 填近似先行）。
- 两个关键裁定随字段冻结落死：
  1. **出口①只携带库内键**（`association_key`）；`external_target_id`/`target_name`
     为场景真值标识，**不得出口**（去真值化纪律，契约规则 5）。
  2. **特征量测语义 = 真值特征经效能约束（SNR/视角覆盖/带宽/驻留）转换的仿真量测**
     ——角度无噪声、RCS 均值无偏（只有 SNR 推定的 std）；字段注释必须明示，防下游
     当作加噪量测使用。

## 1. 现状证据（内部事实，2026-08-18 探索）

| 事实 | 内容 | 证据 |
|---|---|---|
| 特征集结构 | `RirFeatureSet` = 四维观测（RCS/运动/极化/距离像，原生单位 dBsm、m/s、dB、m）+ 逐维 quality[0,1] + valid_feature_mask；逐周期逐 association_key 产生，无状态提取器 | src/remote_identification_radar/recognition/RecognitionTypes.h:38-95 |
| 特征→结论链路 | 观测入滑窗（RirTrackState.window）→ 聚合（运动中位数/其余均值）→ 模板匹配（原生单位逐子特征 z-score）→ 证据积累状态机 | RecognitionTracker.cpp:94-193,253-307；RecognitionMatcher.cpp:18-24 |
| 特征来源 | 场景真值特征（角度-RCS 网格/极化双通道/散射中心）经效能约束转换：RCS 均值不加噪仅推 std_db=3/√snr；极化加 SNR 决定的确定性噪声底；距离像按噪声门删峰；**角度无噪声** | RcsFeatureExtractor.cpp:100-101；PolarizationFeatureExtractor.cpp:69-72 |
| 观测几何现成量 | look_az/el_deg（雷达局部 ENU，**az 自 +x 东起量**）、range_m、snr_db、bandwidth_hz、dwell_sec 在 RirObservationContext | RecognitionTypes.h:23-33；RirController.cpp:181-188 |
| 平台位置缺失 | RirCycleInput 仅 platform_altitude_m，无平台 LLA/ENU 位置——fusion sensor_origin 通道当前**无来源** | RirCycleInput.h:44-56 |
| 输出帧与 replay | RirOutputFrame 仅 recognition_outputs；replay V2 记录内嵌 V1 输出帧表（flatbuffers 加性扩展可行） | RirOutputTypes.h:68-72；schemas/replay/rir_replay.fbs:109-132 |
| fusion feature 通道 | `vector<double>` 无量纲约定、欧氏距离门、**维度不一致不约束**；ESR 先例做单位归一化（Hz→GHz/MHz/ms/µs）；无效维表达无既有约定 | DetectionRecord.h:39；FusionEngine.cpp:339-351；SensorAdapters.cpp:86-90 |
| 库内键先例 | ESR hypothesis_id 直接作为 fusion key 透传（"调用方保证跨源一致"契约） | SensorAdapters.cpp |

## 2. Stage A 证据矩阵

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|---|---|---|---|---|---|---|
| F1 内部特征容纳性 | 四维特征逐周期逐键已结构化存在，出口只需透出路径 | RecognitionTypes.h 四观测结构 | 头文件字段核对（§1） | 全部子值/质量/掩码可零换算映射到公共结构 | 任一维度缺子值或质量定义 | pass |
| F2 去真值化边界 | 出口①可只携带库内键而不携带真值标识 | 契约规则 5 + 航迹快照字段清单 | RirTrackTypes.h:80-110 核对 | external_target_id/target_name 留在归属/调试层，出口仅 association_key | 出口必需真值标识才能对齐 | pass |
| F3 量测语义诚实性 | "真值×效能约束转换"语义可经字段注释+文档冻结 | 提取器实现（§1 第 3 行） | 提取器代码核对 | 公共字段注释明示语义与保真度边界 | 需要加噪链才能出口（则先立特征物理化冻结项） | pass |
| F4 feature 通道对齐 | 11 维固定布局 + 无效维 0 填 + 帧内掩码可先落地，mask-aware 门升级延后 | fusion 特征门语义 + ESR 归一化先例 | 布局表对照（§3.2） | 跨源维度不一致=门不约束（既有语义），RIR 自身跨周期同布局可比 | 0 填近似破坏 RIR 内部跨周期关联 | narrow |
| F5 方位通道换算 | 适配器可做 east→north 参考换算（90°−az，wrap）与单位换算 | 适配器单位换算先例（SBIRS rad→deg、ESR Hz→GHz） | 换算公式核对 | 换算发生在适配器，库内无跨系转换 | 需要库内坐标转换 | pass |
| F6 兼容性 | 输出帧/replay 可加性扩展，出口②零变更 | flatbuffers 加性规则 + replay V1 表结构 | fbs 表结构核对 | 新字段可选、旧 codec 忽略、roundtrip 可测 | replay 字节兼容规则被破坏 | pass |
| F7 OQ-4 修订路径 | 豁免→双产品形态可经 contract 规则 2 修订条款落地 | 契约存量偏离登记机制（规则 7） | 修订文案冻结（§5） | 修订文案随 Stage B 写入 contract.md，OQ-4 条目迁出 | 修订与既有规则冲突 | pass |

## 3. 冻结契约（Frozen Contract）

```text
Proven requirement
  2026-08-18 架构评估采纳：RIR 双产品形态。出口②（识别结论）不可审计、不可多源
  融合、迫使 TARGET-OQ-4 永久豁免；出口①补齐证据链。

Allowed scope（Stage B 实施范围）
  Modules/directories:
    - include/1q/remote_identification_radar/session/RirFeatureMeasurementTypes.h（新）
    - include/1q/remote_identification_radar/session/RirOutputTypes.h（加性追加）
    - src/remote_identification_radar/recognition|session 透出路径 + RirSession.cpp 回填
    - schemas/replay/rir_replay.fbs（V1 输出帧表加可选字段）+ RirReplayFlatbufferCodec
    - include/1q/fusion/SensorAdapters.h + src/fusion/SensorAdapters.cpp
      （新增 AdaptRirFeatureMeasurementsToDetectionRecords）
    - tests：rir 单测/集成 + rir replay roundtrip + fusion sensor_adapters 新用例
    - docs：remote_identification_radar 四件套、fusion algorithms.md（§3 行）、
      target_inference（§4.3 注记）、contract.md 规则 2 修订、open_questions OQ-4 迁出
  Classes/functions:
    - RirFeatureObservations（公共镜像四观测：字段=内部同值，单位后缀命名）
    - RirFeatureMeasurementRecord / RirFeatureMeasurementFrame（§3.1）
    - RirOutputFrame.feature_measurements（可选追加，默认空——旧行为零变化）
    - fusion 适配器（§3.2/§3.3）

Explicitly out of scope
  Public headers:   external_target_id/target_name 不出口（F2）；sensor_origin 不提供
                    （平台位置输入缺失，延后为独立冻结项）
  Cross-module:     不改 target_inference/threat_assessment 代码；不动识别链内部结构
                    （解耦为后续演进项）；不做特征物理化（加噪/电磁链）
  Schema/trace:     replay 仅加性扩展；不新增独立 V3 root（V1 表加可选字段）
  Test thresholds:  既有测试与阈值零修改
  Compatibility:    出口②（RirTrackRecognitionOutput/RirRecognitionResult）零变更

Behavior boundary
  Outputs: 出口①逐周期逐 association_key 一条；全维无效的记录不产生（帧内为空）
  Errors/fallback: 特征提取无效维度按内部现状（valid=false）透出，不虚构
  Lifecycle: 出口①随周期输出，不跨周期持有；进 replay

Acceptance gates
  Build:     rir/fusion 相关目标编译通过（Windows v141+UseEnv 流程）
  Focused:   unit::remote_identification_radar / unit::fusion / replay::remote_identification_radar 全绿
  Contract:  replay roundtrip 新字段往返一致；target_layer_purity_guard 保持绿
  Characterization: 适配器换算单测（east→north、11 维布局、质量聚合）

Non-goals
  平台位置输入字段（origin 通道）、识别链模块内解耦、特征物理化、
  fusion 特征门 mask-aware 升级（各自为独立冻结项，见 §6）
```

### 3.1 出口①字段冻结：RirFeatureMeasurementRecord

| 字段 | 类型/单位 | 语义 |
|---|---|---|
| association_key | uint64 | RIR 内部航迹键（库内键透传；fusion key 语义同 ESR hypothesis_id 先例） |
| features.rcs | {valid, mean_dbsm, std_db, azimuth_variation_db, elevation_variation_db, peak_to_valley_db, aspect_coverage_deg, quality} | dBsm/dB/deg/[0,1]；**std_db 为 SNR 推定不确定度，均值无偏真值取样** |
| features.motion | {valid, speed_m_per_s, altitude_m, acceleration_m_per_s2, turn_radius_m, is_straight, quality} | m/s、m、m/s²、m |
| features.polarization | {valid, energy_difference_db, relative_difference_db, energy_sum_db, quality} | dB；能量和已按 100 km 参考距离补偿 |
| features.range_profile | {valid, length_m, peak_count, peak_energy_concentration, resolution_m, quality} | m、无量纲、[0,1]、m |
| valid_feature_mask | uint8 | 四维有效位掩码（RirRecognitionFeatureDimension 位或） |
| look_az_deg / look_el_deg | deg | 雷达局部 ENU 视线角，**az 自 +x（东）起量**（与 fusion 自北约定不同——换算归适配器，F5） |
| range_m / snr_db / dwell_sec / bandwidth_hz | m / dB / s / Hz | 观测效能上下文 |
| cycle_index / batch_id | uint32/uint64 | 归属 |

**语义注释（必须进公共头 Doxygen）**：特征量测为"场景真值特征经效能约束（SNR/视角
覆盖/带宽/驻留）转换的仿真量测"；角度无噪声、RCS 均值无偏；不得当作加噪量测使用
（保真度升级=特征物理化冻结项）。

### 3.2 fusion 适配器冻结：11 维特征布局（对齐 DetectionRecord.feature）

| 维 | 量 | 换算（适配器内） | 目标量级 |
|---|---|---|---|
| 0 | RCS 均值 | mean_dbsm 原值 | 0–40 |
| 1 | 速度 | speed_m_per_s ÷ 1e3 → km/s | 0.05–1 |
| 2 | 高度 | altitude_m ÷ 1e3 → km | 0–30 |
| 3 | 加速度 | acceleration_m_per_s2 原值 | 0–30 |
| 4 | 转弯半径 | log10(turn_radius_m)，无效=0 | 2–6 |
| 5–7 | 极化三量 | energy_difference/relative_difference/energy_sum_db 原值 | dB 域 |
| 8 | 距离像长度 | length_m 原值 | 1–30 |
| 9 | 峰数 | peak_count 浮点化 | 1–10 |
| 10 | 峰能集中度 | [0,1] 原值 | 0–1 |

- **无效维填 0**（NaN 禁止——毒化欧氏门）；帧内 valid_feature_mask 为权威有效性。
- 跨源（ESR 4 维 vs RIR 11 维）维度不一致 → 特征门不约束（既有语义，恰好符合
  "不同源特征不互判"）。
- **已知近似（F4 narrow）**：0 填会向欧氏距离贡献失真；mask-aware 特征门升级登记为
  后续冻结项。

### 3.3 适配器其余映射冻结

- `AdaptRirFeatureMeasurementsToDetectionRecords(source_id, frame)`：
  跳过全维无效记录；key=association_key（ESR 先例，跨源一致性归调用方）；
  has_bearing=true，bearing_az_deg = wrap(90° − look_az_deg)（east→north 参考换算，
  el 原值）；**has_sensor_origin=false**（平台位置输入缺失，F6 外延后不虚构）；
  verdict=1；quality = 有效维 quality 的加权均值（feature_weights 配置口径，
  缺省等权）。
- 不提供 origin ⇒ RIR 特征记录参与关联与特征门、**不参与三维方位滤波**（与当前
  SBIRS 记录同状态；origin 输入字段为独立后续冻结项）。

## 4. TARGET-OQ-4 裁定修订（豁免 → 双产品）

原裁定（2026-08-17）："识别类传感器"（识别即装备使命、public 输出即识别结论）作为
分层契约规则 2 的显式豁免装备形态；接入走方案 a。

修订裁定（2026-08-18 采纳，Stage B 落地生效）：

1. 识别类传感器为**双产品形态**：识别结论（装备使命产品，保留，形态不变）+
   特征量测帧（合法传感器量测产品，新增）。豁免条款转为正式产品形态——传感器
   不得"只"输出识别结论。
2. 方案 a（调用方键映射）**降级为兼容选项**：特征记录携带库内键 + 方位后，融合层
   可自动关联；双源识别证据融合（RIR 特征 × SBIRS/其他）成为可组合能力。
3. 推演层（target_inference）类型证据通道扩展登记：多源特征证据可经特征量测组成
   （Stage B 仅登记接口可能性，不在 target_inference 内实现新算法）。
4. 条目状态：裁定已采纳（Stage A pass）；待 Stage B 落地（出口①上线 + 契约修订
   写入）后从 open_questions.md 迁出。

## 5. contract.md 规则 2 修订文案（冻结，Stage B 写入）

现行：**传感器产品边界**：威胁评分、目标类型识别结论、轨迹/发射点预测不得作为传感器
public 输出字段（raw output、`*CycleResult`、public DTO 一致适用）。……

修订（在原条文后追加）：「**识别类传感器双产品条款**：以目标识别为装备使命的传感器
（当前：remote_identification_radar）采用双产品形态——识别结论可作为装备使命产品
出口，但**必须同时提供特征量测出口**（带库内键、单位后缀命名、逐维质量与有效掩码、
并明示仿真保真度语义）；只输出结论不输出特征量测的形态不得新增。」

## 6. Stage C 回写（占位）与后续冻结项清单

| 项 | 实际结果 |
|---|---|
| 实现范围 / 验证命令与结果 / 残留风险 | 待 Stage B 填 |

后续冻结项（各自独立立项，本文只登记）：①平台位置输入字段（origin 通道）；
②fusion 特征门 mask-aware 升级；③识别链模块内解耦；④特征物理化（加噪/电磁散射链）。
