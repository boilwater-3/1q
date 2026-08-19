---
Status: active
Last-reviewed: 2026-08-18
Authority: RIR 设计权威入口
Answers: 远程识别雷达模块是什么、和谁交互、设计文档怎么导航
---

# Remote Identification Radar 设计

`remote_identification_radar`（RIR）提供远程目标识别雷达的独立仿真模型：对自持检测-跟踪形成的内部航迹
执行驻留观测，经效能化观测（RCS/运动/双通道极化/宽带一维距离像）与预设特征数据库
加权匹配，输出目标类别与最可能型号，并携带周期识别效能摘要。

RIR 是与机载雷达（AR）**相互独立的另一部雷达装备**，不是 AR 的工作模式或子能力。
本模块由 AR 内被耦合的远程识别子系统（kLrr）解耦而来（2026-08-15 审计：
`docs/review/ar_remote_identification_radar_coupling_audit_2026-08-15.md`；
迁移状态与下一步计算：`docs/review/remote_identification_radar_migration_status_2026-08-15.md`）。

## 模块定位要点

1. **独立装备**：自带 hardware 域（发射机/天线/接收机），自持检测
   （方向图/分项 SINR 账本/统计级 CFAR）、LAPJV 全局最优关联、CV KF/IMM
   双路径滤波与池化生命周期（跟踪升级 N1-N7 已落地）；
   独立输入输出与 replay/trace。与 AR 无任何模块间协作接口，不 include
   任何 AR 头，不消费任何 AR 输出（阶段 2-S 已删除 RirTrackFeed 供给面）。
2. **输入面**：`RirCycleInput` 提供周期戳、必填平台 ECEF（
   `oneq::coordinate::EcefPositionM`，fail-closed 校验）、场景目标
   （含识别特征真值 `aspect_rcs_samples`/`polarization_rcs_samples`/
   `range_rcs_scatterers`；公共 API 为 radar-local ENU，集成层用户侧以 ECEF
   描述目标，适配层边界转换）与 RF 入射链路；环境事实经
   `RirSessionConfig.environment` / 运行期补丁注入。场景目标速度/名称/Swerling
   起伏为自持链路事实；识别只消费效能化观测，场景真值不得直接产生结论。每周期
   波束中心由**库内驻留调度器**派生（扫描策略或指定识别任务，见 boundaries.md
   驻留指向契约），RIR 消费侧只信任并消费给定指向。
3. **输出面（双产品，2026-08-18 Stage B）**：出口②识别结论（`RirRecognitionResult`
   与效能摘要 `RirRecognitionCycleSummary`，形态不变）+ 出口①特征量测帧
   （`RirFeatureMeasurementRecord`：四维特征 + 逐维质量 + 有效掩码 + 库内键 +
   视线角/效能上下文 + 可选平台位置，语义=真值×效能约束转换的仿真量测）。
   归属视图（`RirTrackAttributionRecord`：库内键 ↔ 真值目标对照 + 最小航迹诊断）
   经 `RirCycleResult.track_attributions` 暴露（结果层，不进产品层）。与 AR 威胁
   分类相互独立，不进任何决策帧；指定识别任务状态（`designated_target_id`/
   `designation_*`/`dwell_center_deg`）经 `RirCycleResult` 逐周期暴露。
   fusion 侧由 `AdaptRirFeatureMeasurementsToDetectionRecords` 消费出口①。
4. **配置**：四域（hardware/mission/policy/environment）；policy 域承载
   检测/关联/跟踪/生命周期/识别策略，运行期补丁整域提交；识别作用距离/驻留
   四域归位（任务域）；扫描策略（限位/起点/顺序/步长系数）与指定识别任务
   （目标 ID + 限时窗口）随 mission/patch 配置，库内驻留调度器消费。
5. **数据**：特征数据库为只读 SQLite 基线（schema v1.1，权威 DDL 单源随迁），
   运行期不持有连接；单位纪律：场景 `rcs` 为 m²（SNR 门控），识别 RCS 特征与
   数据库一律 dBsm。
6. **命名**：public 前缀 `Rir*`；issue code 前缀 `rir.validation.*`；
   `ArRecognition*` 前缀废弃，不保留 compat 层。

## 文档导航

- 模块边界、非目标、单位纪律、失败降级、接口不变式、F1/F2 物理保真度边界、
  设计变更规则 → [boundaries.md](boundaries.md)
- 数据流图与状态所有权（数据库/积累/内部航迹/回滚）→
  [data-flow.md](data-flow.md)
- 算法登记表（观测构造/四提取器/积累/匹配/判定）、反直觉点、刻意不实现清单 →
  [algorithms.md](algorithms.md)

跨模块公共规则见 `docs/common/contract.md`。
