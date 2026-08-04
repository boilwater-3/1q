# 远程识别特征数据库 SQLite 迁移规划（下一层）

Status: draft
Date: 2026-08-04
Upstream: `remote_recognition_design(1)(1).md`（§7 数据库与匹配）、`remote_recognition_implementation_plan_2026-08-04.md`（阶段 3）、`remote_recognition_conflict_and_plan_2026-08-04.md`
Flow: evidence-first-freeze-contract（Stage A 证据矩阵 → 冻结契约 → Stage B 计划）

## 1. 背景

远程识别阶段 3 实现了「JSON 文件 + 自研解析器 + 内存结构」的特征数据库。现确认目标交付格式改为
**SQLite 数据库文件**：外部系统以 SQLite 提供识别基线，库内部从 SQLite 加载并保持全部运行期契约。

本规划只做 Stage A + 冻结契约 + Stage B 计划，不进入实现。

## 2. 现状事实（已核实，迁移的证据基础）

| # | 事实 | 代码位置 |
|---|---|---|
| S1 | 存储与加载：`Load(path, db, error)`，candidate 原子替换，`schema_version=="1.0"` 校验 | `src/airborne_radar/recognition/RecognitionFeatureDatabase.cpp:265-366` |
| S2 | 校验规则：字符串非空、`std>0`、`prior>0`、model_id/profile_id/category_id 唯一、category_id 引用存在 | 同上（文件内静态函数集） |
| S3 | 解析器：自研递归下降，深度 64 / EOF 校验 / 转义完整性（contract.md #4）；**唯一消费方是 FeatureDatabase** | `RecognitionJsonParser.{h,cpp}`、grep 证实 |
| S4 | 运行期加载：路径变化或未加载时按需加载；失败保持旧库 + `PROJECT_LOG_ERROR`；成功替换 + `SetActiveDatabaseVersion(version())` | `src/airborne_radar/runtime/ArController.cpp:229-262` |
| S5 | 快照/回滚：`recognition_database_path` 不一致 → reset + clear（释放 DB） | `ArController.cpp:679, 719-721` |
| S6 | 消费面：Matcher::QueryBestMatch 与 Tracker::Update 只读消费 `const RecognitionFeatureDatabase&` | `RecognitionMatcher.h:65`、`RecognitionTracker.h:107` |
| S7 | replay 契约：fbs 仅存 `database_path:string` 与 `active_database_version:string`，**不感知存储格式** | `schemas/replay/airborne_radar_session_replay.fbs:184`、`airborne_radar_replay.fbs:352` |
| S8 | 配置校验：`enabled && database_path.empty()` → error（路径非空是 enabled 前提） | `src/airborne_radar/session/ArSessionConfigBuilder.cpp:188` |
| S9 | 测试构造：两个测试文件用内嵌 JSON 字符串 + `WriteTempJson` | `tests/unit/airborne_radar/ar_recognition_database_test.cpp`、`ar_recognition_integration_test.cpp` |
| S10 | 依赖接入模式：conanfile.py 锁定版本 + `shared=False` 固定；ConanPackages/VendorPackages 双 provider 同契约；PRIVATE 链接不进导出面 | `conanfile.py`、`cmake/project/dependencies/*`、`src/airborne_radar/CMakeLists.txt` |
| S11 | Conan 本地缓存无 sqlite3 包 → bootstrap 时从 conancenter 拉取，版本需锁定 | `conan list "sqlite3/*"` |

## 3. Stage A 证据矩阵

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|---|---|---|---|---|---|---|
| F1 SQLite 迁移需求 | 识别基线交付格式为 SQLite 文件 | 用户确认（2026-08-04） | 无本地可证伪项；作为交付需求接受 | 需求确认即成立 | — | pass（边界 narrow：最小替换） |
| F2 运行期契约保持 | 原子替换、失败保持旧库、路径触发重载、快照释放语义不因存储格式改变 | S4/S5 | 迁移后跑 scenario/integration 测试 | 原测试全绿、行为零变化 | 任一语义变化 | pass |
| F3 消费面零改动 | Matcher/Tracker 只读消费 `const DB&`，不感知存储格式 | S6 | 编译 + grep | 两文件零改动 | 需改 Matcher/Tracker | pass |
| F4 replay 字节契约不受影响 | fbs 只存路径/版本字符串 | S7 | 迁移后跑 replay roundtrip | 全绿、fbs/codec 零改动 | 需改 fbs/codec | pass |
| F5 校验规则完整性 | std>0、prior>0、唯一性、category 引用、schema 版本校验全部保留 | S2 | database_test 迁移后全绿 | 失败用例在新存储一一对应 | 任一校验丢失 | pass |
| F6 SQLite 是加载期读取器 | 加载后数据库全量驻留内存，连接可关闭；运行时无需连接 | 现有架构（DB 为纯内存结构） | 代码审查 | Load 返回后无 sqlite3_* 残留调用 | 需持有连接 | pass（关键简化） |
| F7 JSON 解析器去留 | 迁移后 RecognitionJsonParser 无消费方 → 删除；examples json_reader 独立保留 | S3 + grep | 删除后 grep 零引用 | 编译通过、零引用 | 有其他消费方 | pass（删除） |
| F8 依赖接入可行 | Conan sqlite3 包 + 双 provider 同契约 + PRIVATE 链接 | S10、zlib/highfive/jsbsim 先例 | 双 preset 构建 | 两路径均构建通过 | 单路径失败 | pass |
| F9 运行时查询/增量/并发写 | 基线只读、整体替换、加载期一次性即够 | 无需求证据 | — | — | 无证据证明需要 | reject（YAGNI） |
| F10 Windows SQLite 验收 | Windows 下 Conan 包可用 | Windows 为未验收脚手架 | — | — | 现有 Windows 非验收状态 | defer |

Stage A 结论：**8 pass（含 1 narrow）/ 1 reject / 1 defer**，进入冻结契约。

## 4. 冻结契约

### Proven requirement

识别特征库从 JSON 文件迁移到 SQLite 文件；数据模型（database → categories → models → profiles → 四类模板）
与全部校验规则保持；运行期加载/失败/回滚/replay 契约保持；`RecognitionFeatureDatabase` 对外接口不变。

### Allowed scope

- 模块/目录：`src/airborne_radar/recognition/`（改 `RecognitionFeatureDatabase.{h,cpp}`；删 `RecognitionJsonParser.{h,cpp}`）
- 类/函数：`RecognitionFeatureDatabase::Load`（签名不变，实现换 SQLite）；sqlite 加载实现（文件内静态函数或新 .cpp）
- 依赖接入：`conanfile.py`、`cmake/project/dependencies/ConanPackages.cmake`、`cmake/project/dependencies/VendorPackages.cmake`、`scripts/fetch_third_party`（如涉及）、`src/airborne_radar/CMakeLists.txt`
- 测试：`tests/unit/airborne_radar/ar_recognition_database_test.cpp`（重写）、`ar_recognition_integration_test.cpp`（DB 构造改造）、新增 sqlite 测试 helper
- 文档：`docs/airborne_radar/algorithms.md`（数据库条目 + [evidence]）、本规划 Stage C 回写

### Explicitly out of scope

- Public headers：`include/1q/` 零结构改动；仅允许 `ArRecognitionConfig.h:63` `database_path` 注释措辞更新（"JSON 路径" → "SQLite 路径"，字段名/类型/语义不变）
- 跨模块类型：无
- Schema/trace/replay：fbs/codec 零改动
- 测试阈值/skip：不调整
- 兼容层：**禁止** JSON/SQLite 双路径、禁止读取旧 JSON 文件的迁移转换器
- 运行时连接：禁止在 `RecognitionFeatureDatabase` 中持有 SQLite 连接（F6）
- `examples/common/json_reader`：不动（examples 层独立）

### Behavior boundary

- 输入：`database_path` 语义变为 SQLite 文件路径；空串仍表示未配置（S8 校验不变）
- 输出：`IsLoaded()/version()/database_id()/models()/categories()` 语义不变
- 错误/回退：打开失败、损坏文件、校验失败 → `Load` 返回 false + error（含路径与表/字段上下文），保持旧库；
  不抛异常（sqlite3 为 C API 无异常，错误码转状态字符串）
- 生命周期：加载期打开（只读）→ 读 meta/表 → 校验 → 关闭连接 → 填充 candidate → 原子提交；成功后无连接驻留
- 版本语义：`schema_version` 以 meta 值存字符串（== "1.0" 校验同 JSON 版）；`version()`/`database_id()` 来自 meta

### Acceptance gates

- Build：`llvm-ninja-release-local` + `llvm-ninja-debug-local` 双 preset 全量构建
- Focused tests：`ar_recognition_database_test` / `ar_recognition_integration_test` / `ar_recognition_feature_test` / `ar_recognition_scenario_test` 全绿
- Contract tests：replay roundtrip（RecognitionFields / DefaultState / SessionConfig）零改动通过
- 全量：ctest 全绿（当前基线 53/53 不降）

### Non-goals

- 海量基线按需查询 / 运行时连接（F9 reject）
- 增量更新、热替换、并发读写（F9 reject）
- 外部写库工具（DDL 文档化即可）
- 性能基准（加载为一次性初始化，无规模需求）

## 5. 设计要点（Stage B 前定稿）

### 5.1 SQLite schema

```sql
PRAGMA foreign_keys = ON;

CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);
-- 行：schema_version = '1.0'（校验同 JSON 版字符串比较）
--     database_id / version = 现有语义字符串

CREATE TABLE categories(
  category_id TEXT PRIMARY KEY,
  prior REAL NOT NULL CHECK(prior > 0));

CREATE TABLE models(
  model_id TEXT PRIMARY KEY,
  category_id TEXT NOT NULL REFERENCES categories(category_id),
  prior REAL NOT NULL CHECK(prior > 0));

CREATE TABLE profiles(
  profile_id TEXT PRIMARY KEY,
  model_id TEXT NOT NULL REFERENCES models(model_id),
  -- applicability
  min_snr_db REAL,
  max_range_resolution_m REAL,       -- NULL/0 表示不限
  -- rcs
  rcs_mean_dbsm REAL, rcs_std_db REAL, rcs_azimuth_variation_db REAL,
  rcs_elevation_variation_db REAL, rcs_minimum_aspect_coverage_deg REAL,
  -- motion（speed/altitude/acceleration 的 mean/std；turn_radius 用 mean_log10/std_log10）
  motion_speed_mean REAL, motion_speed_std REAL, ...,
  turn_radius_mean_log10 REAL, turn_radius_std_log10 REAL,
  -- polarization / range_profile（同样拍平为列）
  ...);
```

要点：
- **模板可选字段用可空列表达**（对应 JSON 版 `Has()` 缺省语义）；`std` 可空列非 NULL 时加载校验 `> 0`（S2 保持）
- 唯一性/引用由 SQLite 约束（`PRIMARY KEY`/`REFERENCES`）兜底 + `Load` 内显式校验（与 JSON 版一致，错误信息含表/列）
- `profile_id` 在 JSON 版要求模型内唯一——SQLite 用全表 `PRIMARY KEY`（更严）或 `(model_id, profile_id)` 复合主键保持原语义；**定稿选择：复合主键 `(model_id, profile_id)`**，与 JSON 版"模型内唯一"语义完全一致

### 5.2 加载流程

```
open(SQLITE_OPEN_READONLY) → PRAGMA foreign_keys → 读 meta → schema_version 校验
→ 读 categories → 读 models → 读 profiles（按 model 分组）
→ 显式校验（std>0/prior>0/引用/复合主键唯一由约束保证）
→ 关闭连接 → 填充 candidate → loaded_=true 原子提交
失败任意一步 → error 含路径+上下文，返回 false，调用方保持旧库（S4）
```

### 5.3 依赖接入（S10 模式）

| 文件 | 改动 |
|---|---|
| `conanfile.py` | `_BASE_DEPS` 加 `sqlite3/<锁定版本>`；`default_options` 加 `"sqlite3/*:shared": False` |
| `ConanPackages.cmake` | `find_package(sqlite3 CONFIG REQUIRED)`；**不进 `ONEQ_LINK_DEPENDENCIES`**（PRIVATE 实现细节） |
| `VendorPackages.cmake` | vendored sqlite3 源码 `add_subdirectory`（参照 zlib"必须链接二进制"模式）产出 `sqlite3::sqlite3` 同契约 target；`scripts/fetch_third_party` 同步 |
| `ProjectInstall.cmake` | **零改动**（PRIVATE 依赖不出现在下游 `find_dependency`） |
| `src/airborne_radar/CMakeLists.txt` | `target_link_libraries(airborne_engine PRIVATE sqlite3::sqlite3)`；源列表移除 `RecognitionJsonParser.cpp`、加入新 sqlite 源 |

### 5.4 测试构造

- 新增测试 helper：`WriteTempSqlite(path, kCreateSql)`（内嵌 DDL+INSERT，参照现有 `WriteTempJson` 模式）
- `ar_recognition_database_test.cpp` 重写：有效库加载 / 缺 meta / 坏 schema_version / 零 std / 坏引用 / 重复 id 各失败用例一一对应 JSON 版
- `UnicodeEscape` 等 parser 专属用例随 parser 删除（F7）
- `ar_recognition_integration_test.cpp`：`kDatabaseJson` → 等价 SQLite 构造；`database_path_` 后缀 `.json` → `.db`

## 6. Stage B 实现计划（按提交切分）

**B1 依赖接入（独立提交，无编译消费）**
1. `conanfile.py` 加 sqlite3
2. `ConanPackages.cmake` / `VendorPackages.cmake` + `scripts/fetch_third_party`
3. 验证：bootstrap + configure 通过

**B2 存储迁移（核心提交）**
4. `RecognitionFeatureDatabase.h` 注释更新（接口不变）
5. `RecognitionFeatureDatabase.cpp`：`Load` 换 SQLite 实现，校验函数集改造（作用于行数据）
6. 删除 `RecognitionJsonParser.{h,cpp}`；`src/airborne_radar/CMakeLists.txt` 源列表 + 链接
7. `ArRecognitionConfig.h:63` 注释措辞更新

**B3 测试改造**
8. 新增 sqlite 测试 helper
9. `ar_recognition_database_test.cpp` 重写
10. `ar_recognition_integration_test.cpp` DB 构造改造

**B4 文档回写**
11. `docs/airborne_radar/algorithms.md` 数据库条目 + `[evidence]`
12. 本规划 Stage C 回写（实际结果、残留风险）

## 7. 验收门（Stage C）

- 双 preset 构建 + ctest 全量（基线 53/53 不降）
- replay 测试零改动通过（fbs/codec 未动，F4 证据）
- grep 确认无 JSON 兼容层残留、无 `RecognitionJsonParser` 引用

## 8. 非目标与后续观察（reject/defer 汇总）

- **F9 reject**：运行时查询/增量更新/并发写——无需求证据，禁止（YAGNI）
- **F10 defer**：Windows SQLite 验收——与既有 Windows 脚手架状态一致，不单独扩验收
- **后续观察项**：若真实基线进入万级型号规模，再以新 freeze item 评估运行时连接/按需查询

## 9. 待确认决策点（实现前）

1. **打开模式**：只读 `SQLITE_OPEN_READONLY`（推荐，匹配"只读基线"语义）——确认后冻结
2. **复合主键**：`profiles` 用 `(model_id, profile_id)` 复合主键保持 JSON 版"模型内唯一"语义（推荐）
3. **sqlite3 版本**：bootstrap 时以 conancenter 可用版本锁定（建议 3.4x.x 系列，实现时确定具体版本号）
4. **测试 helper 落点**：`tests/unit/airborne_radar/` 下新增（跟随现有测试目录结构）

## 10. Stage C Result（2026-08-04，实现后回写）

### Implemented scope

**B1 依赖接入**
- `conanfile.py`：`_BASE_DEPS` 加 `sqlite3/3.53.4`；`default_options` 加 `"sqlite3/*:shared": False`；`requirements()` 接线
- `cmake/project/dependencies/ConanPackages.cmake`：`find_package(SQLite3 CONFIG REQUIRED)`（conan-center 包名 `SQLite3`、target `SQLite::SQLite3`，与规划中的 `sqlite3::sqlite3` 命名修正）；不进 `ONEQ_LINK_DEPENDENCIES`
- `cmake/project/dependencies/VendorPackages.cmake`：vendored sqlite3（amalgamation 单文件编译 + `SQLite::SQLite3` IMPORTED 同契约 target，zlib 同构）
- `scripts/fetch_third_party.bat`：`fetch_one sqlite3 https://github.com/sqlite/sqlite.git v3.53.4`

**B2 存储迁移**
- `src/airborne_radar/recognition/RecognitionFeatureDatabase.cpp`：`Load` 换 SQLite 实现（只读打开 → meta 校验 → 表加载 → 显式校验 → 关闭连接 → candidate 原子提交）；RCS std 错误字段名保持 `std_db`（对齐 JSON 版断言）
- `src/airborne_radar/recognition/RecognitionFeatureDatabase.h`：注释更新，接口不变
- 删除 `RecognitionJsonParser.{h,cpp}`（唯一消费方已随迁移消失）
- `src/airborne_radar/CMakeLists.txt`：源列表移除 parser；`airborne_engine` PRIVATE 链接 `SQLite::SQLite3`
- `src/CMakeLists.txt`：公共库聚合显式 `target_link_libraries(${PROJECT_CORE_TARGET} PRIVATE SQLite::SQLite3)`（OBJECT 库 PRIVATE 依赖不传播，需在此纳入链接闭包——与 ZLIB 模式一致，实现时发现并补齐）
- `include/1q/airborne_radar/config/ArRecognitionConfig.h`：`database_path` 注释措辞（JSON → SQLite）

**B3 测试改造**
- 新增 `tests/unit/airborne_radar/RecognitionSqliteTestUtil.h`（`WriteTempSqlite` + 共享 `kRecognitionSchemaSql`）
- `ar_recognition_database_test.cpp` 重写：SQLite 构造，失败用例一一对应（缺 models 表用局部 DDL；std==0；坏 category 引用用 `PRAGMA foreign_keys=OFF` 构造；schema 9.9）；UnicodeEscape 随 parser 删除
- `ar_recognition_integration_test.cpp` / `ar_recognition_scenario_test.cpp`：JSON → SQLite 构造
- `tests/cmake/partitions/Unit.cmake` / `Integration.cmake`：airborne_radar partition 加 `LINK_LIBS SQLite::SQLite3` + `INCLUDE_DIRS`
- 测试路径字符串 `.json` → `.db`（`ar_session_config_builder_test` / `ar_runtime_patch_mapper_test` / `ar_replay_codec_roundtrip_test`）

**B4 文档回写**
- `docs/airborne_radar/algorithms.md`：识别数据库登记行更新（SQLite schema v1.0 + 加载期只读读取器）
- `docs/airborne_radar/boundaries.md`：F1 通道定义措辞不绑定已删除的 `polarization_channels` 字段（该字段 JSON 版即文档性，加载器从不消费，grep 证实）

### Validation

- `bash scripts/bootstrap_conan.sh llvm-ninja-release-local`：pass（sqlite3/3.53.4 从 conancenter 下载安装）
- `cmake --preset llvm-ninja-release-local`：pass
- `cmake --build --preset llvm-ninja-release-local`：pass（engine 冒烟先行，链接闭包补齐后全量通过）
- `ctest --preset llvm-ninja-release-local -j 4`：**53/53 pass**（基线不降；识别 unit 35 用例 + integration + replay 全绿）
- 无 JSON 兼容层残留；`RecognitionJsonParser` 零引用（grep 证实）

### Residual risks

- **vendored 路径未本机验证**：`PACKAGE_MANAGER=none` 面向 Windows 未验收脚手架，`VendorPackages.cmake` 的 SQLite 段与 conan provider 同契约但未经真实构建（Windows runner 上需随既有脚手架一并验收）
- **sqlite3/3.53.4 版本锁定**：conancenter 当前最新；后续升级需重跑 bootstrap 并回归识别测试
- **只读打开语义**：`SQLITE_OPEN_READONLY` 使加载期误写文件立即失败——与"只读基线"语义一致，属预期行为
- **schema 文档性**：`polarization_channels` 等 JSON 版文档性字段未迁移进 SQLite schema（加载器本就不消费），外部写库工具如需保留需自行约定 meta 键

### Follow-up freeze items

- ~~flatbuffers NotNested/EndTable 违规群（跨模块 codec）~~：**已处理**（2026-08-04，独立提交）。
  Stage A 定位：ESR/EOS codec 共 8 处 `add_xxx(fbb.CreateVector(...))`（父 table builder 打开期间
  建向量，debug 断言崩溃 / release 静默损坏 vtable 偏移）+ 测试 4 处（common 测试字段偏移 0
  覆写 buffer 开头、ESR roundtrip 3 处同模式）。修复：向量创建全部前置到 builder 打开之前
  （与 AR co_site_paths 先例同模式）；参数内 CreateVector（`CreateXxx(fbb, ..., CreateVector)`）
  本就合法（参数求值先于函数体 StartTable），等价前置仅统一模式、字节不变。
  验证：debug 3 个崩溃测试（common unit / ESR replay / cross_domain integration）全修复，
  release 53/53 保持；项目未上线、无存量 trace，不做字节兼容（边界处理优先）。
- 真实基线进入万级型号规模时，重新评估"加载期全量读入 vs 运行时连接/按需查询"（当前 F9 reject 的再进入条件）
- Windows no-Conan 路径整体验收（F10 defer 状态不变）
