---
Status: draft
Date: 2026-08-15
Plan-Baseline: `main` @ `96de367c`
Authority: 第一阶段实施计划；归属判定以
  `docs/review/ar_remote_identification_radar_coupling_audit_2026-08-15.md`（下称
  《审计》）为准。若本计划与库实现冲突，以库为准。
---

# 远程识别雷达解耦 第一阶段计划：新建 `remote_identification_radar` 模块（先建后用）

## 0. 策略与目标

总策略（已确认）：**阶段 1 新建远程识别雷达模块并测试通过；阶段 2 再分离 AR 中的
远程识别耦合**（strangler 模式——新模块先落地验证，AR 原实现暂不动，两套并存由
等价性测试锁定一致性，阶段 2 删除 AR 侧）。

阶段 1 完成定义（DoD）：

1. 新模块 `remote_identification_radar` 具备本库五模块同构的完整解剖（§2），
   `release-local` 下 `unit/integration/contract/replay` 四类测试全绿；
2. 新模块**不 include 任何 AR 头**（含 internal `src/airborne_radar/**`），
   构建无 AR 依赖；
3. AR 模块零代码改动：`include/1q/airborne_radar/**`、
   `src/airborne_radar/**`、`schemas/replay/airborne_radar*.fbs`、
   `tests/{unit,integration,replay}/airborne_radar/*.cpp`、`docs/airborne_radar/**`
   逐字节不动，且 AR 聚焦测试全绿；
4. 资产三件（识别 DDL、建库工具、示例库）迁移至新模块目录，AR 识别测试**内容零
   改动**仍全绿（仅指针级路径更新，见 §4 白名单）；
5. 等价性对比测试建立：同场景同输入下 AR kLrr 识别输出与新模块输出逐字段一致
   （§5.4），作为阶段 2 删除 AR 侧的保真基线；
6. `check_public_api_boundary.cmake`、`check_cross_domain_naming.cmake`、
   `test_layout_guard` 通过。

## 1. 命名与治理（已定稿 2026-08-15）

| 项 | 定稿 |
|---|---|
| 目录 | `include/1q/remote_identification_radar/`、`src/remote_identification_radar/` |
| 命名空间 | `remote_identification_radar`（config/session/recognition 子域同 AR 模式） |
| public 前缀 | `Rir*`（`RirSession`/`RirCycleInput`/`RirRecognitionConfig`/`RirSceneTarget`…） |
| issue code 前缀 | `rir.validation.*`（对应迁移 AR 的 `ar.validation.recognition_*` 五码） |
| 模块入口头 | `remote_identification_radar.hpp`（聚合 config 域 + session 稳定门面，不含 observability 工具头——遵守入口头守则） |
| 组件目标 | `rir_engine`（识别算法，链接 `SQLite::SQLite3` PRIVATE）+ `rir_core`（会话/配置/适配器，依赖轻） |
| 测试分区域 | `remote_identification_radar`（分区标签 `unit::remote_identification_radar` 等） |
| 工作模式 | `RirWorkMode { kStby = 0, kIdentify = 1 }`（对应原 kLrr 的"是否处于识别驻留"语义；阶段 2 删除 `ArWorkMode::kLrr` 后无值域冲突） |

`ArRecognition*` 前缀废弃，不保留 compat 层（对齐 `Radar*`→`Ar*` 一次性迁移先例）。

## 2. 新模块解剖与类型迁移映射

### 2.1 四域配置

| Rir 域 | 内容 | 迁移来源 | 语义变更 |
|---|---|---|---|
| `RirHardwareConfig` | `transmitter`/`receiver`/`antenna`（engineering 结构**副本**，见 §3.3）+ 无识别参数 | AR `ArRecognitionStaticContext` 借用链 | 无（副本平移） |
| `RirMissionConfig` | `work_mode`（`RirWorkMode`） | `ArWorkMode::kLrr` 语义 | 模式归属归位 |
| `RirPolicyConfig` | `RirRecognitionPolicy`：`enabled`、`min_confirmed_hits`、`accumulation_window_sec`、`min_observation_count`、`acceptance_score`、`minimum_margin`、`result_hold_sec`、`max_range_m`、`recognition_dwell_sec`、`feature_weights`、`database_path` | `ArRecognitionConfig` 全字段平移 | **无**（max_range/dwell 暂留 policy 域，四域归位列为阶段 2 后评估项，保证等价性对比输入直映射） |
| `RirEnvironmentConfig` | 空占位 struct | 无（当前识别链不消费环境服务，snr 由雷达方程+热噪声底自算） | 新增；`boundaries.md` 明示"不引入未消费字段"（对齐 AR 死输入审计教训） |

### 2.2 session 输入输出面

| Rir 类型 | 内容 | 迁移来源 |
|---|---|---|
| `RirSceneTarget` | `external_target_id`、`position_x/y/z`、`rcs`（m²，探测标量——SNR 门控用）、`range_m`、三组识别真值（`aspect_rcs_samples`/`polarization_rcs_samples`/`range_rcs_scatterers`，类型改名 `RirAspectRcsSample` 等） | `ArSceneTarget` 子集 + 识别真值字段 |
| `RirTrackFeedEntry` | `association_key`、`external_target_id`、`status`（confirmed 判定）、`position_x/y/z`、`velocity_x/y/z`、`estimation_uncertainty_trace`、`target_type` | `TrackStateSnapshot` 消费字段子集（**以 `MotionFeatureExtractor::Extract` 实际读取字段为准逐字段核对**） |
| `RirCycleInput` | 周期戳（cycle_index/batch_id/sim_time）+ 平台状态（海拔/姿态）+ `RirSceneTargetList` + `RirTrackFeed` | `ArCycleInput` 子集 + 识别真值 |
| `RirCycleResult` | status/issues（问题列表模型，对齐 `session_contract.md` 规则 9/14）+ 每航迹 `RirRecognitionResult` + `RirRecognitionCycleSummary` | `ArCycleResult` 识别字段 |
| `RirRecognitionResult` 等 | `RirRecognitionState`/`Category`/`FeatureDimension`/`FeatureScores`/`Result`/`CycleSummary`（**枚举取值顺序与 AR 完全一致**，保证等价性对比与 replay 语义） | `ArRecognitionResult.h` 全类型 |
| `RirSession` | `Create`/`Step`/`StepWithResult`/运行时 patch 提交（policy 整域，回滚语义）；非执行周期不复用上一帧 | AR session 公共契约模式 |
| `RirIssueCodes`/`RirInputValidation`/`RirSessionConfigBuilder`/`RirSessionConfigValidation`/`RirProfileConstants`/`RirRuntimeConfigPatch` | 同构五模块骨架 | AR 对应件（校验段迁识别五码，code 改 `rir.validation.*`） |
| `RirExternalInputAdapter`/`RirCycleOutputAdapter` | 真值拷贝/结论装配 | AR 对应件识别段 |

### 2.3 内部实现（`src/remote_identification_radar/`）

| 目录 | 内容 | 迁移来源与改造点 |
|---|---|---|
| `recognition/` | 观测构造 + 四提取器 + 特征库加载 + 匹配 + 跟踪积累 | `src/airborne_radar/recognition/` 17 文件**复制改写**：namespace/include 换 `Rir*`；`RecognitionObservationContext`→`RirObservationContext`；消费 `RirTrackFeedEntry` 而非 `TrackStateSnapshot` |
| `runtime/` | `RirController`：主链 = 库加载（`UpdateRecognitionRuntime` 逻辑）+ 识别执行（`RunRecognitionCycle` 逻辑）+ 波束/候选选择（`PrepareLrrPointing`/`IsBetterLrrCandidate` 迁入，消费 `RirTrackFeedEntry::target_type`）+ 快照回滚 | `ArController` 识别段（§审计 3.2）复制改写；**不再有 AR 波束注入**——识别驻留指向由本模块自持（阶段 1 仅保留候选选择逻辑，输出驻留决策到 `RirCycleResult`，不驱动任何外部波束） |
| `internal/radar_equations.h/.cpp` | `ComputeEchoPower_dBW`/`ComputeThermalNoisePower_W` + engineering 三结构（`TransmitterConfig`/`ReceiverConfig`/`AntennaConfig`）**副本** | `src/airborne_radar/signal/detection/RadarEquations.*` + `config/SignalEngineeringConfig.h` 所需最小集；文件头注明"副本来源：… @ 96de367c，阶段 3 评估 common 化" |
| `session/` | 组装根、适配器、校验、replay codec、trace | AR 对应件识别段复制改写 |
| `CMakeLists.txt` | `rir_engine`（recognition/internal）+ `rir_core`（runtime/session）+ `rir_replay.fbs`/`rir_session_replay.fbs` | 镜像 `src/airborne_radar/CMakeLists.txt` 模式；`rir_engine` 链 `SQLite::SQLite3` PRIVATE |

### 2.4 replay/trace

- `schemas/replay/rir_replay.fbs`：`RirRecognitionResultV1`、`RirRecognitionCycleSummaryV1`、结果帧与会话状态（含 `active_database_version`）——表结构与 AR 版同构（等价性对比/阶段 2 迁移验证用）；
- `schemas/replay/rir_session_replay.fbs`：`RirRecognitionPolicyV1`、`RirRecognitionFeatureWeightsV1`；
- `RirTraceSession`/`RirReplaySession`：对齐公共 trace/replay 模式；replay 逐周期比较识别结果（浮点容差 `1e-5f`），`database_version` 入 replay state。

## 3. 骨架与构建接线（文件级步骤）

1. 目录树：`include/1q/remote_identification_radar/{config,session}/`、
   `src/remote_identification_radar/{config,runtime,recognition,internal,session}/`、
   `docs/remote_identification_radar/`、`schemas/replay/rir_*.fbs`、
   `schemas/remote_identification_radar/`、`tests/{unit,integration,replay,contract}/` 对应目录；
2. `src/CMakeLists.txt`：`add_subdirectory(remote_identification_radar)`（唯一 1 行；62-64 行
   SQLite 链接闭包与注释**不动**——阶段 2 更新注释归属）；
3. `conanfile.py` **不动**（sqlite3 已是 base dep，`SQLite::SQLite3` 双 provider 现成）；
4. `tests/contract/check_public_api_boundary.cmake`：新增
   `RIR_PUBLIC_PRIMARY_HEADERS`/`RIR_SESSION_HEADERS` 两个集合并入
   `EXPECTED_PUBLIC_HEADERS`；入口头加入 `MODULE_ENTRY_HEADERS_WITH_EXPLICIT_TOOLING`；
5. 测试分区注册：
   - `tests/cmake/partitions/Unit.cmake`：GLOB `unit/remote_identification_radar/*_test.cpp`
     + `_oneq_add_*_partition(TYPE unit DOMAIN remote_identification_radar ...)`；
   - `tests/cmake/partitions/Integration.cmake`：partition + 注入
     `ONEQ_RIR_EXAMPLE_DATABASE_PATH`（指向新示例库路径）；
   - `tests/cmake/partitions/Contract.cmake`/`Replay.cmake`：各加一行 partition 注册；
6. 验证骨架成立：空模块编译通过 + `check_public_api_boundary`、
   `check_cross_domain_naming`、`test_layout_guard` 全绿后再填内容。

## 4. 资产迁移与阶段 1 指针白名单

### 4.1 资产三件迁移（识别雷达自有资产，属"建新模块"而非"动 AR"）

| 现路径 | 新路径 | 配套指针更新 |
|---|---|---|
| `schemas/recognition/recognition_feature_database.sql` | `schemas/remote_identification_radar/recognition_feature_database.sql` | `tests/CMakeLists.txt:28-36` 的 `file(READ)`/`CMAKE_CONFIGURE_DEPENDS` 路径；生成头输出名 `recognition_feature_database_schema.h` **保持不变**（AR 识别测试零内容改动仍编译） |
| `tools/recognition_db_builder.py` | `tools/remote_identification_radar_db_builder.py` | 工具内 schema/输入/输出默认路径常量；CLI 参数接口不变 |
| `examples/configs/recognition/`（含 `recognition_database_input.json`、`target_feature_database_v1.1.db`） | `examples/configs/remote_identification_radar/` | `tests/cmake/partitions/Integration.cmake` 的 `ONEQ_RECOGNITION_EXAMPLE_DATABASE_PATH` 路径指针（变量名**保留**，阶段 2 删除） |

DDL 单源不变式全程保持：**移动而非复制**，三条消费方（加载器语义、测试生成头、
建库工具）始终指向同一文件。

### 4.2 阶段 1 允许触碰的既有文件（指针级白名单，全部列出）

`src/CMakeLists.txt`（+1 行）、`tests/CMakeLists.txt`（schema 路径）、
`tests/cmake/partitions/{Unit,Integration,Contract,Replay}.cmake`（新分区 + 变量路径）、
`tests/contract/check_public_api_boundary.cmake`（新模块白名单）、
`examples/configs/README.md` 与 `examples/README.md`（recognition 段指针）、
`examples/component_attachment/logger/logger_i18n.h`（**新增** `rir.*` 五码翻译，
旧五条 `ar.validation.recognition_*` **保留**至阶段 2）。

### 4.3 阶段 1 禁止触碰（逐字节不动）

`include/1q/airborne_radar/**`、`src/airborne_radar/**`、
`schemas/replay/airborne_radar*.fbs`、`tests/unit|integration|replay/airborne_radar/*.cpp`
（含 7 个识别测试与 `RecognitionSqliteTestUtil.h`）、`docs/airborne_radar/**`、
`conanfile.py`、`tests/cmake/TestTargets.cmake`（分区新增会自动生成目标，如需手工项
再以最小 diff 补）。

## 5. 测试迁移与新增

### 5.1 迁移改写（旧文件不动，新建 Rir 版）

| 新文件 | 来源蓝本 | 改写点 |
|---|---|---|
| `tests/unit/remote_identification_radar/RirSqliteTestUtil.h` | `RecognitionSqliteTestUtil.h` | include 与 `kRecognitionSchemaSql` 常量名 |
| `rir_recognition_database_test.cpp` | `ar_recognition_database_test.cpp` | 类型/命名空间换 `Rir` |
| `rir_recognition_feature_test.cpp` | `ar_recognition_feature_test.cpp` | 同上；`TrackStateSnapshot` → `RirTrackFeedEntry` 构造 |
| `rir_recognition_integration_test.cpp` | `ar_recognition_integration_test.cpp` | kLrr patch → `RirWorkMode::kIdentify`；`ArSession` → `RirSession` |
| `tests/integration/remote_identification_radar/rir_recognition_example_database_test.cpp` | `ar_recognition_example_database_test.cpp` | 加载器类型 + `ONEQ_RIR_EXAMPLE_DATABASE_PATH` |
| `rir_recognition_scenario_test.cpp` | `ar_recognition_scenario_test.cpp` | 场景/会话构造换 Rir |
| `rir_recognition_us_military_scenario_test.cpp` | `ar_recognition_us_military_scenario_test.cpp` | 同上 + 新 env 变量 |
| `tests/replay/remote_identification_radar/rir_replay_codec_roundtrip_test.cpp` | `ar_replay_codec_roundtrip_test.cpp` 识别段 | Rir codec 全字段 roundtrip（含默认态） |

### 5.2 新增解剖契约测试

- `rir_session_config_validation_test`：识别五码校验（`rir.validation.*`）+ 四域默认值；
- `rir_runtime_patch_test`：policy 整域覆盖与回滚；
- `rir_session_contract_test`：非执行周期不复用上一帧、关机/拒绝语义（对齐
  `session_contract.md` 规则）；
- `rir_recognition_database_test` 内新增：DDL 单源加载校验（全字段加载用例同 AR 版）。

### 5.3 场景构造复用

`ar_recognition_scenario_test` 的内联 SQL 构造（`kScenarioDatabaseSql`）与
`RecognitionSqliteTestUtil.h` 已内嵌 DDL——Rir 版测试继续经生成头
`recognition_feature_database_schema.h` 使用同一 DDL 字符串（路径迁移后自动生效）。

### 5.4 等价性对比测试（阶段 1 核心保真守卫）

`tests/integration/cross_domain/ar_rir_recognition_equivalence_test.cpp`：

- 同一场景（场景目标表 + 确认航迹 + 同一 SQLite 库）双跑：
  1. AR：`ArSession` 启用识别（`kLrr` patch），N 周期 `Step`，取
     `output_frame.tracks[i].recognition`；
  2. Rir：相同硬件数值构造 `RirSessionConfig`（hardware 三结构与 AR engineering
     数值一致），`RirCycleInput` 由 AR 场景目标子集映射 + AR 航迹输出映射
     （`TrackStateSnapshot` → `RirTrackFeedEntry` 字段投影），同库同权重，N 周期；
- 断言：逐航迹 `state/category/model/confidence/best_score/runner_up_score/
  feature_scores 八分量/valid_feature_mask/observation_count/accumulation_sec/
  database_version` 一致（浮点容差 `1e-5f`），摘要计数一致；
- 意义：锁定"复制改写"未引入语义漂移；阶段 2 删除 AR 侧后此测试退役（或反转方向）。

## 6. 文档

1. `docs/remote_identification_radar/` 四件套（design/boundaries/data-flow/algorithms）：
   以《审计》§1.1 功能归属清单 + 否决项为底稿；`boundaries.md` 明示：四域解剖、
   `RirEnvironmentConfig` 空占位理由、单位纪律（dBsm vs m²）、ENU 帧约定、
   失败降级、replay 契约、F1/F2 保真度边界（随文迁移）；
2. `docs/review/` 计划与审计文档互链（审计 §6 已加链接）；
3. examples 层指针（§4.2 白名单内两项 README + i18n）。

## 7. 实施顺序（步骤与验证门）

> 执行进度（2026-08-15）：步骤 1（骨架）与步骤 2（public 类型 + 四域配置 +
> 校验）已完成：21 个 public 头、`rir_core` 组件（`RirSessionConfigValidation.cpp`）、
> 守卫接线（public 白名单 / cross-domain naming / docs structure / 四类测试分区）、
> `docs/remote_identification_radar/design.md`；验证：契约守卫 24/24 全绿、
> 全库 Release 构建通过。另修复基线预存白名单漂移
> （`coordinate/inertial_transform.h` 缺失，SBIRS 提交 ff8e6b6e 引入）。
> 构建环境注记：本机 msbuild+v141 需 `vcvarsall x64 -vcvars_ver=14.16` +
> `/p:UseEnv=true` 才能解析 Windows SDK 路径（预存环境问题，与本次改动无关）。
>
> 执行进度（同日续）：步骤 3-6 已完成：识别内部实现 17 文件平移改写（Rir 命名空间/
> 类型，零 AR 依赖）、内部雷达方程副本（`RirRadarEquations`，来源 commit 注明）、
> `RirController`/`RirSession`/`RirInputValidation` 落地、单元 26 例 + 集成 28 例
> （场景/交付库/美方 15 型号）全绿、资产三件随迁（DDL/建库工具/示例库）且 DDL
> 单源保持、AR 聚焦回归全绿、契约守卫 25/25（补 test_layout_guard 域白名单）。
> 提交：`066a5fe2`。剩余：步骤 5（replay/trace + fbs + roundtrip）、步骤 7
> （等价性对比）、步骤 8（模块文档三件 + examples 指针收尾）。

| 步 | 内容 | 验证门（release-local） |
|---|---|---|
| 1 | 骨架：目录 + CMake 组件 + 空模块编译 + 白名单 + 分区注册 | build 通过；`ctest -R "contract"` 守卫全绿（白名单/naming/layout） |
| 2 | public 类型 + 四域配置 + 校验（`Rir*` DTO 全套） | `unit::remote_identification_radar`（配置/校验/契约新测试）绿 |
| 3 | internal 平移：`recognition/` + 雷达方程副本 + 候选选择；迁 database/feature 两测试 | `rir_recognition_database_test`、`rir_recognition_feature_test` 绿 |
| 4 | session 组装（`RirController`/`RirSession`/适配器）；迁 integration 三测试 | `integration::remote_identification_radar` 绿 |
| 5 | replay/trace + fbs + roundtrip 测试 | `replay::remote_identification_radar` 绿 |
| 6 | 资产三件迁移 + 指针白名单更新 | **AR 聚焦回归**：`unit::airborne_radar` + `integration::airborne_radar` + `replay::airborne_radar` 全绿（内容零改动验证） |
| 7 | 等价性对比测试（§5.4） | `integration::cross_domain` 新测试绿 |
| 8 | 文档四件套 + examples 指针 + i18n | 文档检查（无构建影响） |
| 9 | DoD 终检 | §0 六条逐项核对；聚焦 ctest 全绿 |

验证命令遵循 CLAUDE.md：**release-local** 预设；聚焦
`ctest --preset release-local -R "<域|分区>"`（如
`-R "remote_identification_radar"`、`-R "unit::airborne_radar"`）；
**不跑 debug-local 全量**；完整 release 全量留给阶段 2 末尾 `/completeness-review`。

## 8. 风险与决策记录

| # | 风险/决策点 | 处置 |
|---|---|---|
| R1 | `RirTrackFeedEntry` 字段集与 `TrackStateSnapshot` 偏差导致运动特征漂移 | 以 `MotionFeatureExtractor::Extract` 实际消费字段为准逐字段核对（`estimation_uncertainty_trace` 语义写入模块契约）；等价性测试兜底 |
| R2 | 雷达方程/engineering 结构副本漂移 | 副本文件头注明来源 commit；阶段 3 评估 common 化（候选：`src/common/` 雷达方程共享） |
| R3 | `max_range_m`/`recognition_dwell_sec` 四域归位延后 | 阶段 1 保持 policy 域平移（语义零变更），归位列为阶段 2 后评估项并写入文档决策记录 |
| R4 | 双份识别代码并存期行为漂移 | 等价性测试锁定 + 阶段 1 期间 AR 侧冻结（禁止触碰清单 §4.3）+ 阶段 2 尽快删除 |
| R5 | DDL/示例库移动破坏 AR 识别测试 | 内容零改动、指针级白名单、步骤 6 全量 AR 聚焦回归门 |
| R6 | 公共 session 契约适用性（三写/问题列表/非执行周期不复用） | 新模块直接对齐 `session_contract.md` 规则 9/14，`RirCycleResult` 单一 `issues` 列表 |
| R7 | `check_cross_domain_naming` 对新前缀的约束 | 命名定稿（§1）先行；Rir 与既有域无同名概念冲突预期 |
| R8 | 示例库 `.db` 二进制移动 | git rename 追踪，无内容变更 |

## 9. 阶段 2 预告（本计划不展开）

按《审计》§3 文件级清单删除 AR 侧识别耦合（public 字段/枚举、执行链、replay 表、
issue code、SQLite 链接注释、嵌入测试段、文档章节），同步执行《审计》§9 七条验收
标准；新增双模块集成契约（航迹供给接口时序、生命周期耦合、威胁类型字符串契约）；
等价性测试退役；末尾跑 `/completeness-review` 完整 release 验证。
