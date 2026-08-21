---
Status: draft
Date: 2026-08-21
Review-Baseline: `main` @ `6776523e`（merge: feature/ar-rir-drift-fixes）
Authority: AR 仿真真值归属统一（信封对照表 + 产品字段降级 + CI 硬门槛）的决策记录
  与冻结契约。本文承载 ArCycleResult.track_attributions 字段冻结、产品帧真值字段
  deprecated 语义冻结与 attribution_mounting_guard 门槛规则；产品字段的物理回收
  （去真值化收回，TARGET-OQ-1 Stage A）不在本文范围。非规范性记录，若与库实现
  冲突，以库为准。
---

# AR 航迹归属统一：信封对照表 + 产品字段降级 + 硬门槛

## 0. 结论速览

- 背景：EOS/SBIRS/RIR 的仿真真值归属挂信封通道（`detection_attributions` /
  `track_attributions` 对照表），AR 是唯一把真值键（`external_target_id`/
  `target_name`）直接印在产品航迹（`TrackStateSnapshot`）上的历史特例。
- 本次裁定（2026-08-21）：**不改产品行为，改权威路径**——
  1. AR 信封补齐 `ArCycleResult.track_attributions`（与 RIR 同构），成为真值归属
     的权威路径；
  2. 产品帧上的 `external_target_id`/`target_name` 降级为 **deprecated 遗留
     （sim-only）**，新代码不得以其为关联依据；物理回收归 TARGET-OQ-1 Stage A；
  3. 新增 CI 源码守卫 `attribution_mounting_guard`：公共头中真值标识符只允许
     出现在注册表内，AR 产品遗留注册表**冻结**——特例不再开第二次。
- 理由（架构评估结论）：产品通道只应承载系统"挣来的"信息；真值键进产品使评测
  无法自证盲评、归属错误不可事后修复、实测数据无对应物造成语义漂移。"track 是
  系统级估计"最多推出该带系统自铸 track ID，推不出该带场景真值 ID。

## 1. 现状证据（2026-08-21 探索）

| 事实 | 内容 | 证据 |
|---|---|---|
| AR 真值注入链 | 场景输入 target_id/name → 坐标变换透传 → 量测 → 命中回填 track → 快照导出 → 产品帧；非跟踪器推导，是场景真值透传 | ArRadarFrameTransform.cpp:120-121；TrackMeasurementProcessing.cpp:51-52；TrackLifecycleManager.cpp:542-547；TrackStateSnapshotEmitter.cpp:138-178 |
| AR 信封无归属 | ArCycleResult（L56-108）无任何 attribution 字段 | ArCycleResult.h |
| RIR 模板 | RirTrackAttributionRecord{association_key, external_target_id, target_name, hit_count, ENU 位置, 速度}，RirController 逐周期对全部航迹快照构建，仅 kCompleted 挂载 | RirOutputTypes.h:83-92；RirController.cpp:792-852；RirCycleResult.h:57-58 |
| 融合核心不依赖真值字段 | AR 适配器 fusion key = association_key；EOS/SBIRS key=0（去真值化纪律） | SensorAdapters.cpp:99-125 |
| 库内"就地 join"消费者 | 生命周期记录器/排除记录器/调试视图按 external_target_id 扫 output_frame | ArTrackLifecycleRecorder.cpp:18-26；ArExclusionCauseRecorder.cpp:81-128；ArTrackOutputDebugViewBuilder.cpp:21-46 |
| 契约特例登记 | session_contract.md Attribution 挂载表 AR 行"本次不搬迁（去真值化收回由后续独立工作处理）"；TARGET-OQ-1 登记冻结公共 API | session_contract.md:389；open_questions.md:377-404 |
| 演进政策 | 项目未上线，直接演进无兼容分支；fbs 加性字段（表尾追加），root_type/file_identifier 不变 | rir_replay.fbs:1-13；airborne_radar_replay.fbs 头注 |

## 2. 冻结契约（Frozen Contract）

```text
ArTrackAttributionRecord（信封通道，挂 ArCycleResult.track_attributions）：
  association_key    u64     AR 内部航迹关联键（与产品帧同键同序）
  external_target_id u64     场景真值目标 ID（0 = 未提供，哨兵原样保留）
  target_name        string  场景真值目标名（可空）
规则：
  R1 覆盖范围 = 产品导出航迹（output_frame.tracks，去重后），逐条对应、同序；
     运动学/命中数不在此重复出口（信封已内嵌产品帧副本）。
     【已知限制】被去重抑制的内部航迹不在覆盖内；扩展需动 signal 管道
     （SelectOutputTracks 之前取全量快照），登记为后续工作。
  R2 仅 kCompleted 周期填充；校验拒绝/关机/中止周期为空列表，不复用上一周期
     （五模块统一规则）。
  R3 replay：fbs 表尾追加可选向量（flatbuffers 加性规则），root_type=ArCycleInputV3
     与 "ARC3" 标识不变；旧记录解码为空列表。
  R4 产品帧 external_target_id/target_name 行为零变化（冻结公共 API，TARGET-OQ-1）；
     注释层标记 DEPRECATED(sim-only)，不加 [[deprecated]] 属性（约 20 个测试文件
     在用，告警噪音无意义）。
  R5 CI 硬门槛 attribution_mounting_guard（tests/contract/check_attribution_mounting.cmake）：
     公共头含真值标识符（external_target_id/target_name/*Attribution*）必须在
     三类注册表之一——TRUTH_INPUT（场景输入/调用方指定键/诊断码）、
     ENVELOPE_ATTRIBUTION（*CycleResult/*OutputTypes 的 AttributionRecord 与
     DebugView/LifecycleRecorder/ExclusionCauseRecorder 观测层）、
     AR_LEGACY_PRODUCT_TRUTH（冻结：TrackStateSnapshot.h/ArTrackOutput.h/
     ArExternalOutputAdapter.h，不得新增）。
```

## 3. 实施清单

| 变更 | 位置 |
|---|---|
| 记录类型 + 信封字段 | ArOutputTypes.h（ArTrackAttributionRecord）；ArCycleResult.h（track_attributions） |
| 生产端（会话装配期派生） | ArSession.cpp BuildCompletedCycleResult，从 completed.output_frame.tracks 逐条构建 |
| replay schema + codec | airborne_radar_replay.fbs（ArTrackAttributionRecord 表 + ArCycleResultV3 表尾追加）；ArReplayFlatbufferCodec.cpp encode/decode |
| 单元测试 | tests/unit/airborne_radar/ar_track_attribution_test.cpp（逐条对应 + 非执行周期空列表） |
| replay roundtrip | tests/replay/airborne_radar/ar_replay_codec_roundtrip_test.cpp（含 0 哨兵记录往返） |
| 边界合同测试 | ar_output_boundary_contract_test.cpp 头注更新为新契约状态 |
| CI 守卫 | tests/contract/check_attribution_mounting.cmake + ContractGuards.cmake 注册（LABELS contract） |
| 文档 | session_contract.md Attribution 挂载表（AR 双挂载 + 规则 5）；open_questions.md TARGET-OQ-1 当前边界补注；docs/airborne_radar/boundaries.md 两通道条款 |

## 4. 明确不做（本轮）

- 不移除产品字段、不动真值注入链（ArRadarFrameTransform → TrackLifecycleManager
  → TrackStateSnapshotEmitter）。
- 不迁移库内消费者（ArTrackLifecycleRecorder、ArTrackOutputDebugViewBuilder、
  TrackOutputQueries 实现、ar_sensor_component 示例）到信封对照表——它们消费的
  是 deprecated 遗留字段，行为不变；迁移随字段回收（Stage A）一并处理。
- precision_evaluation 纳入 AR、batch_validation 归属完整性检查：后续工作。

## 5. 验证矩阵

| 验证 | 方式 | 结果 |
|---|---|---|
| 守卫拦截未知真值头 | 临时探针头 include/1q（阴性测试，已清理） | FATAL 如期 |
| 守卫注册表与磁盘一致 | cmake -P 手动 + ctest -L contract | pass |
| 归属表逐条对应 + 非执行周期 | unit ar_track_attribution_test | pass |
| replay 往返（含 0 哨兵） | ar_replay_codec_roundtrip_test | pass |
| 产品行为零变化 | 存量 AR unit/integration/replay 全量回归 | pass |
