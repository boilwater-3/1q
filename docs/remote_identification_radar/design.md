---
Status: active
Last-reviewed: 2026-08-15
Authority: RIR 设计权威入口
Answers: 远程识别雷达模块是什么、和谁交互、设计文档怎么导航
---

# Remote Identification Radar 设计

`remote_identification_radar`（RIR）提供远程目标识别雷达的独立仿真模型：对重点航迹
执行驻留观测，经效能化观测（RCS/运动/双通道极化/宽带一维距离像）与预设特征数据库
加权匹配，输出目标类别与最可能型号，并携带周期识别效能摘要。

RIR 是与机载雷达（AR）**相互独立的另一部雷达装备**，不是 AR 的工作模式或子能力。
本模块由 AR 内被耦合的远程识别子系统（kLrr）解耦而来（2026-08-15 审计：
`docs/review/ar_remote_identification_radar_coupling_audit_2026-08-15.md`；
第一阶段解耦计划：`docs/review/ar_remote_identification_decoupling_phase1_plan_2026-08-15.md`）。

## 模块定位要点

1. **独立装备**：自带 hardware 域（发射机/天线/接收机），自管识别驻留指向，
   独立输入输出与 replay/trace；与 AR 只存在"航迹供给"这一模块间接口——
   消费外部雷达（如 AR）公开输出的已确认航迹（`RirTrackFeedEntry`）。
   **2026-08-15 需求方二次定案：AR 与 RIR 完全独立、无模块间协作接口，航迹
   供给接缝在阶段 2-S 退役，RIR 自持检测 + 轻量关联**（见
   `docs/review/rir_signal_chain_capability_boundary_2026-08-15.md` §6 与
   `docs/review/remote_identification_radar_phase2_plan_2026-08-15.md` v2）。
2. **输入面**：`RirCycleInput` 提供周期戳、平台海拔、场景目标（含识别特征真值
   `aspect_rcs_samples`/`polarization_rcs_samples`/`range_rcs_scatterers`）
   与航迹供给；识别只消费效能化观测，场景真值不得直接产生结论。
3. **输出面**：识别结论（`RirRecognitionResult`）与效能摘要
   （`RirRecognitionCycleSummary`）为独立输出模型，与 AR 威胁分类相互独立，
   不进任何决策帧。
4. **配置**：四域（hardware/mission/policy/environment）；识别策略
   （`RirRecognitionPolicy`）为 `ArRecognitionConfig` 的整域平移（语义与默认值
   不变，保证等价性测试直映射）；`max_range_m`/`recognition_dwell_sec` 的四域
   归位（任务域）列为阶段 2 后评估项。
5. **数据**：特征数据库为只读 SQLite 基线（schema v1.1，权威 DDL 单源随迁），
   运行期不持有连接；单位纪律：场景 `rcs` 为 m²（SNR 门控），识别 RCS 特征与
   数据库一律 dBsm。
6. **命名**：public 前缀 `Rir*`；issue code 前缀 `rir.validation.*`；
   `ArRecognition*` 前缀废弃，不保留 compat 层。

## 文档导航

- 模块边界、非目标、单位纪律、失败降级、接口不变式、F1/F2 物理保真度边界、
  设计变更规则 → [boundaries.md](boundaries.md)
- 数据流图、状态所有权（数据库/积累/回滚）、与 AR 的航迹供给关系 →
  [data-flow.md](data-flow.md)
- 算法登记表（观测构造/四提取器/积累/匹配/判定）、反直觉点、刻意不实现清单 →
  [algorithms.md](algorithms.md)

跨模块公共规则见 `docs/common/contract.md`。
