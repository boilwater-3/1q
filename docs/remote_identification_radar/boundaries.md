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
- **独立输入面（阶段 2-S 已落地）**：与 AR 无任何模块间接口。输入为场景目标
  （含速度/名称/Swerling 起伏/识别特征真值）+ RF 入射链路 + 环境快照；内部
  航迹由 RIR 自持检测与轻量跟踪生产。`RirTrackFeed` 公开供给已删除。
- **驻留指向（阶段 2-S）**：波束指向由 RIR 自管；驻留候选排序消费内部航迹，
  语义为"未识别优先 + 斜距次近"（威胁等级输入随独立性消失）。RIR 不驱动任何
  外部雷达波束。
- **自持检测链（阶段 2-S 已接线；跟踪升级 2-T N1-N7 已完成）**：需求所列九项
  信号链能力（天线方向图仿真、回波/干扰/噪声功率计算、四项处理增益、恒虚警
  检测）界定为 **RIR 自持检测链**（检测 → LAPJV 全局最优关联 → CV KF/IMM
  双路径滤波 → 池化生命周期 → 内部航迹 → 识别积累）；检测量测仅内部消费，
  不对外发布点迹；战术决策/ECCM/对外点迹仍为非目标。逐项归属与阶段切分见
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
7. 阶段 3 common 化已完成：LAPJV / 雷达方程 / 天线方向图已收敛到 `src/common/`，
   RIR 保留薄适配层，不引入 AR 头。

## 非目标（否决项）

1. ISAR / 二维距离-多普勒像、微动特征、在线学习/自适应权重、实时外部数据库联网、
   信号级 IQ/全波散射求解。
2. 非 `kIdentify` 模式激活识别链路；威胁分类混入识别输出；以场景真值直接产生结论。
3. 暴露内部识别类型为 public SPI；process-wide 识别全局状态。
4. 战斗级跟踪之外的关联决策/战术决策：RIR 自持跟踪为 **LAPJV 全局最优关联 +
   CV KF/IMM 双路径滤波 + 池化生命周期**（2026-08-15 跟踪升级 N1-N7 已落地；
   IMM 为 confirmed 命中激活，`enable_imm_lifecycle` 缺省关闭）。战术决策、
   ECCM、对外点迹/航迹输出仍否决；检测判决不解释为对外
   "目标发现"事件，检测量测不出 public 面。
5. 波束控制：不通过本模块 API 控制外部雷达波束（驻留指向调度未落地前不虚构接口）。

## 单位纪律

- `RirSceneTarget::rcs`（m²，探测链标量）与识别 RCS 特征/数据库（dBsm）显式区分，
  不得混用；数据库 units 表 `rcs == 'dBsm'`，声明其他单位即拒绝。
- 特征单位：速度 m/s、高度 m、加速度 m/s²、转弯半径 log10(m)、极化 dB、距离 m。

## ENU 帧约定

- 识别高度观测 = 平台海拔 + 内部航迹 `position_z`；`position_z` 为雷达局部
  ENU 切平面上向分量，由场景目标位置经自持滤波后回写。
- 场景目标 `position_x/y/z` 同帧；`range_m` 为斜距（>0 或带非零位置）。
- 视角样本网格（`aspect_az_deg`/`aspect_el_deg`）为雷达局部视线角；RCS 插值为
  最近邻（不强制覆盖），覆盖下限属数据库 profile 级适用条件，由匹配阶段判定。

## 失败降级与状态机

1. 库未加载/版本不兼容 → `kDisabled`（不影响自持检测跟踪与其余能力）。
2. 分数/分差不足 → `kUnknown` 或仅大类；运动维度不能单独确认型号。
3. 内部航迹 lost 回收/无观测保持期 → 结论按 `result_hold_sec` 后置 `kStale`；
   关联键单调分配且不回收复用，键重分配天然等于新目标。
4. 退出 `kIdentify` → 清空积累，结论进入保持期；再次进入从零积累。
5. 关机 → 不触碰识别状态，结论在恢复后按保持期过期；校验拒绝不推进积累。
6. 数据库加载失败保持原库（路径变更时按需加载，失败仅降级并记录日志）。

## 接口不变式

- 识别配置经 `RirRuntimeConfigPatch::has_policy` 整域提交（无叶子级 recognition
  patch 字段）；任务域经 `has_mission` 整域提交，`has_work_mode` 叶子可覆盖
  mission 内工作模式。
- 公共枚举加性扩展（不重排既有值，replay 字节兼容）；新增类别须同步
  `RirTracker::CategoryToPublic` 映射与 `RirRecognitionCategory`。
- replay 逐周期比较识别结果（浮点容差 `1e-5f`），`database_version` 与检测随机
  种子入 `RirSessionReplayState`，不一致即 failure；replay schema 为 V2 破坏性
  版本，旧 V1 记录显式拒绝。
- 识别配置校验：`rir.validation.recognition_*` 五码（权重/路径/门限/计数/时间范围）；
  其中任务域作用距离/驻留字段为 `mission.max_range_m`/`recognition_dwell_sec`（四域归位后）。

## 识别子模型的物理保真度边界（F1/F2）

识别链路在效能级观测之上引入两条**识别专用更高保真观测路径**，与自持检测链
物理口径**不逐项对账**（阶段 2-S 后本模块已有探测链，F1/F2 仍只服务于识别
维度有效性与质量）：

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
5. 输入面/RF/环境字段变化须同步 `RirInputValidation` 新 issue code、契约白名单
   与场景测试；不再存在 AR 航迹供给契约。
6. 验证范围：`unit::remote_identification_radar`、`integration::remote_identification_radar`、
   `replay::remote_identification_radar`、`integration::cross_domain`。

[evidence: tests/unit/remote_identification_radar/]
[evidence: tests/integration/remote_identification_radar/]
[evidence: tests/integration/cross_domain/ar_rir_recognition_equivalence_test.cpp]
