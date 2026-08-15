---
Status: active
Last-reviewed: 2026-08-15
Authority: RIR 模块级边界、非目标与设计变更规则
Answers: RIR 有哪些模块级禁令与边界、哪些非目标、单位纪律与失败降级契约
---

# Remote Identification Radar 模块边界

本文承载 RIR 的模块级边界、非目标与设计变更规则。算法级边界见
[algorithms.md](algorithms.md)。

## 装备前提与模块定位

RIR 是与机载雷达（AR）**相互独立的另一部雷达装备**，不是 AR 的工作模式或子能力
（2026-08-15 审计定案）。本模块由 AR 内被耦合的远程识别子系统（kLrr）解耦而来：

- **独立硬件**：自带 hardware 域（`RirHardwareConfig`：发射机/天线/接收机），
  效能级 SNR 由模块内 `RirRadarEquations` 自算，不引用 AR 内部实现。
- **航迹供给（现状，待退役）**：与 AR 只存在"航迹供给"这一模块间接口——消费
  外部雷达公开输出的已确认航迹（`RirTrackFeedEntry`），由调用方编排（如
  "AR `Step()` → 供给航迹 → RIR `Step()`"）。**2026-08-15 需求方二次定案：AR 与
  RIR 完全独立、无模块间协作接口，本接缝与 `RirTrackFeed` 公开输入在阶段 2-S
  退役**（识别积累改挂内部航迹，见
  `docs/review/rir_signal_chain_capability_boundary_2026-08-15.md` §6 与
  `docs/review/remote_identification_radar_phase2_plan_2026-08-15.md` v2）。
- **驻留指向（现状，待迁移）**：识别驻留波束调度不迁移（阶段 1 不消费，阶段 2
  后评估项）；本模块当前只消费航迹供给，不驱动任何外部波束。阶段 2 改为消费
  内部航迹、排序语义"未识别优先 + 斜距次近"（威胁等级输入随独立性消失）。
- **自持检测链（阶段 2 落地中）**：需求所列九项信号链能力（天线方向图仿真、
  回波/干扰/噪声功率计算、四项处理增益、恒虚警检测）界定为 **RIR 自持检测链**
  （检测 → 轻量关联/滤波/生命周期 → 内部航迹 → 识别积累）；检测量测仅内部
  消费，不对外发布点迹；战斗级跟踪（IMM/LAPJV/航迹池）为非目标。逐项归属与
  阶段切分见
  `docs/review/rir_signal_chain_capability_boundary_2026-08-15.md`。
- **replay 会话配置**：阶段 1 只提供周期结果记录编解码（`rir_replay.fbs`）；
  会话配置 replay 与 trace 事件流列为阶段 2 评估项。

## 与 common 契约的关系

RIR 遵守 `docs/common/contract.md` 与 `docs/common/session_contract.md`：

1. public API 只暴露稳定 session/config/input/output/validation/replay DTO 门面；
   `RirController`、识别内部类型不通过 public header 暴露。
2. `RirSessionConfigBuilder` 是薄封装（整域赋值 + `Build()` 返回副本）。
3. 输出遵守三层模型：系统输出（`RirOutputFrame`）、结构化执行结果
   （`RirCycleResult`）、replay 视图分离。
4. 周期语义：非执行周期不复用上一帧；校验拒绝 `kRejectedInvalidInput` +
   明细 issues；关机 `kPoweredOff` 只推进世界时间；统一问题列表
   （规则 14，`RirIssueList`，code 前缀 `rir.validation.*`）。
5. 电源状态单源：`RirSessionConfig::sensor_enabled`，补丁入口
   `RirRuntimeConfigPatch::has_sensor_enabled`（COMMON-OQ-4 对齐）。
6. 跨域形状契约：`ONEQ_SENSOR_SESSION_CONTRACT` 锚定
   `RirSession::Step/StepWithResult` 签名。

## 非目标（否决项）

1. ISAR / 二维距离-多普勒像、微动特征、在线学习/自适应权重、实时外部数据库联网、
   信号级 IQ/全波散射求解。
2. 非 `kIdentify` 模式激活识别链路；威胁分类混入识别输出；以场景真值直接产生结论。
3. 暴露内部识别类型为 public SPI；process-wide 识别全局状态。
4. 航迹滤波/关联/决策（**待阶段 2-S 改写**，见《能力边界》§6）：现行——航迹由
   外部雷达供给，本模块不实现探测、关联、跟踪或战术决策；也不把供给航迹解释为
   自身探测结果。阶段 2-S 后——自持轻量跟踪（单目标 KF + 门限关联 + 计数生命
   周期），战斗级跟踪（IMM/LAPJV/航迹池）、关联决策与战术决策仍否决；检测判决
   不解释为对外"目标发现"事件。
5. 波束控制：不通过本模块 API 控制外部雷达波束（驻留指向调度未落地前不虚构接口）。

## 单位纪律

- `RirSceneTarget::rcs`（m²，探测链标量）与识别 RCS 特征/数据库（dBsm）显式区分，
  不得混用；数据库 units 表 `rcs == 'dBsm'`，声明其他单位即拒绝。
- 特征单位：速度 m/s、高度 m、加速度 m/s²、转弯半径 log10(m)、极化 dB、距离 m。

## ENU 帧约定

- 识别高度观测 = 平台海拔 + `RirTrackFeedEntry::position_z`；`position_z` 为雷达
  局部 ENU 切平面上向分量（含平台姿态旋转），由供给方按帧约定构造
  （等价性测试以 `TryEcefToEnu` 复算 AR 内部帧对齐）。
- 场景目标 `position_x/y/z` 同帧；`range_m` 为斜距（>0 或带非零位置）。
- 视角样本网格（`aspect_az_deg`/`aspect_el_deg`）为雷达局部视线角；RCS 插值为
  最近邻（不强制覆盖），覆盖下限属数据库 profile 级适用条件，由匹配阶段判定。

## 失败降级与状态机

1. 库未加载/版本不兼容 → `kDisabled`（不影响航迹供给与其余能力）。
2. 分数/分差不足 → `kUnknown` 或仅大类；运动维度不能单独确认型号。
3. 航迹丢失供给（保持期无观测）→ 结论按 `result_hold_sec` 后置 `kStale`；
   `association_key` 重分配（`hit_count` 回落）视为新目标。
4. 退出 `kIdentify` → 清空积累，结论进入保持期；再次进入从零积累。
5. 关机 → 不触碰识别状态，结论在恢复后按保持期过期；校验拒绝不推进积累。
6. 数据库加载失败保持原库（路径变更时按需加载，失败仅降级并记录日志）。

## 接口不变式

- 识别配置经 `RirRuntimeConfigPatch::has_policy` 整域提交（无叶子级 recognition
  patch 字段）。
- 公共枚举加性扩展（不重排既有值，replay 字节兼容）；新增类别须同步
  `RirTracker::CategoryToPublic` 映射与 `RirRecognitionCategory`。
- replay 逐周期比较识别结果（浮点容差 `1e-5f`），`database_version` 入
  `RirSessionReplayState`，不一致即 failure。
- 识别配置校验：`rir.validation.recognition_*` 五码（权重/路径/门限/计数/时间范围）。

## 识别子模型的物理保真度边界（F1/F2）

识别链路在效能级观测之上引入两条**识别专用更高保真观测路径**，与任何探测链
物理口径**不逐项对账**（本模块现行无探测链，该约定随迁自 AR 边界文档；阶段 2-S
落地自持检测链后本条口径按
`docs/review/rir_signal_chain_capability_boundary_2026-08-15.md` §6 复核改写）：

1. **F1 双通道极化**：场景目标 `polarization_rcs_samples`（dBsm）经同一雷达方程
   与 SNR 噪声底派生；通道定义（H/V）由数据库固定（meta `polarization_channels`）。
2. **F2 距离像相干叠加**：仅消费场景侧 `range_rcs_scatterers` 真值列表；散射中心级
   峰值判定为效能级简化（粗距离单元下不合并峰标识，仅投影能量）。

上述两条仅存在于 `src/remote_identification_radar/recognition/`。

## 识别特征数据库契约（schema v1.1）

- 自描述只读基线：meta 必填六键、units 必填七量纲且 `rcs == 'dBsm'`。
- 权威 DDL 单源：`schemas/remote_identification_radar/recognition_feature_database.sql`，
  C++ 加载器、测试生成头（`recognition_feature_database_schema.h`）、建库工具
  （`tools/remote_identification_radar_db_builder.py`）共用；禁止第二份 DDL。
- 加载期只读读取器，运行期不持有 SQLite 连接。
- 版本策略：`schema_version` 为 `major.minor`；major 破坏性变更加载器拒绝。

## 设计变更规则

1. 新增/删除/改变 public SPI 时，同步本文档集、consumer tests 与
   `check_public_api_boundary.cmake` 白名单。
2. 任何新增 runtime patch 字段，明确整域/叶子归属并接入提交语义。
3. 观测构造/提取/匹配/判定语义变化，必须同步 algorithms.md 与
   `tests/unit|integration/remote_identification_radar/` 对应测试。
4. 输出字段变化保持三层分离；replay 表变更评估字节兼容。
5. 与 AR 的航迹供给契约变化（字段增删/帧约定）须同步等价性测试
   （`tests/integration/cross_domain/ar_rir_recognition_equivalence_test.cpp`）。
6. 验证范围：`unit::remote_identification_radar`、`integration::remote_identification_radar`、
   `replay::remote_identification_radar`、`integration::cross_domain`（等价性）。

[evidence: tests/unit/remote_identification_radar/]
[evidence: tests/integration/remote_identification_radar/]
[evidence: tests/integration/cross_domain/ar_rir_recognition_equivalence_test.cpp]
