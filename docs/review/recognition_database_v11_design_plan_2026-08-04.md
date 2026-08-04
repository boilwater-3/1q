# 识别特征数据库合理设计规划（v1.1：语义分组、自描述、权威 DDL 单源、建库工具）

Status: draft
Date: 2026-08-04
Upstream: `remote_recognition_design(1)(1).md`（§7 数据库与匹配）、`remote_recognition_sqlite_plan_2026-08-04.md`（v1.0 迁移，已实施）
Flow: evidence-first-freeze-contract（Stage A 证据矩阵 → 冻结契约 → Stage B 计划）

## 1. 背景

v1.0 SQLite 迁移已完成（`remote_recognition_sqlite_plan_2026-08-04.md` §10）：JSON 结构机械拍平为 4 表
（meta/categories/models/profiles，profiles 30 列，NULL 表达模板缺席），加载期只读读取器，运行期契约不变。

本规划的目标是**把 schema 从"JSON 拍平"推进为"合理设计"**：按用户确认的方向（2026-08-04）——
语义分组表 + 元数据补齐（schema v1.1），以及 DDL 权威资产 + 建库工具（消除两处事实源、提供交付途径）。

本规划只做 Stage A + 冻结契约 + Stage B 计划，不进入实现。

## 2. 现状事实（已核实，设计决策的证据基础）

| # | 事实 | 代码位置 |
|---|---|---|
| S1 | v1.0 schema 是 JSON 的机械拍平：4 表，profiles 30 列拍平，NULL=模板组缺席；模板组（rcs/motion/polarization/range_profile）不是独立实体 | `tests/unit/airborne_radar/RecognitionSqliteTestUtil.h:28-54` |
| S2 | 设计意图（§7）：数据库是**自描述**的只读基线——顶层含 `created_utc`、`polarization_channels`、`polarization_energy_reference`、`units`（7 个量纲）；类别/型号含 `display_name`；applicability 含 aspect 方位/俯仰区间 | `remote_recognition_design(1)(1).md:271-435`（§7.2/7.3/7.4） |
| S3 | **缺口**：旧 JSON 加载器消费面恰等于 v1.0 SQLite 表面（37 个字段，git 核实 `634ab3dc~1`）；S2 的自描述元数据全部未入库——`units` 丢失后数值语义只剩隐式约定，`display_name`/aspect 区间丢失 | 旧 `RecognitionFeatureDatabase.cpp`（git 历史）vs 设计文档 §7.2 |
| S4 | DDL 无权威资产：schema 只在测试工具内联，加载器 SELECT 列名另写一处（`RecognitionFeatureDatabase.cpp:452-467`）——两处事实源，迁移期已发生一次列名错位事故（`rcs_mean_dbsm`/`rcs_std_db` 命名不一致，靠报错修复） | 测试 util vs 加载器 SELECT |
| S5 | 无建库工具、无演进机制（`schema_version=="1.0"` 硬编码单字符串）；设计文档 §7.1 仍写 `.json` 文件命名（文档漂移）；仓库**零真实 .db 文件**（`examples/configs/recognition/` 目录不存在） | 设计文档:275、`find . -name "*.db"` 为空 |
| S6 | 消费模式：加载期只读读取器 + 全量内存驻留；Matcher/Tracker 只读消费 `const RecognitionFeatureDatabase&`——查询性能无关，**自描述性/校验完整性/演进能力**才是"合理"的判据 | `RecognitionMatcher.cpp:144-170`、`RecognitionFeatureDatabase.h` |
| S7 | 全仓库仅 airborne_radar 识别库使用 SQLite（grep 证实），无跨模块数据库需求——不建通用库层 | grep `sqlite3` 全仓库 |
| S8 | 结构体（内部头，非 `include/1q`）：CategoryEntry/Model/Profile 无 display_name/aspect 字段；模板组为内嵌 struct，缺席由加载器判定 | `src/airborne_radar/recognition/RecognitionFeatureDatabase.h:59-81` |
| S9 | 仓库先例：`.fbs` 资产位于根 `schemas/`（非 AGENTS.md 所述 tools/）；python3 工具先例存在（`scripts/pre-commit-review.py`、`tools/*.py`）；无 `configure_file` 先例（但为 CMake 标准做法） | `schemas/replay/*.fbs`、`tools/` |
| S10 | 校验规则基线（v1.0 保持）：字符串非空、std>0、prior>0、id 唯一、引用存在；错误信息含路径+表/字段上下文；无异常（C API 错误码转字符串） | `RecognitionFeatureDatabase.cpp`（静态函数集） |

## 3. Stage A 证据矩阵

| Freeze item | Hypothesis | Evidence source | Probe/Test | Pass criterion | Rejection criterion | Decision |
|---|---|---|---|---|---|---|
| D1 元数据自描述（units/created_utc/通道约定入库） | 数据库作为交付格式必须自描述——数值语义（dBsm 非 m²、m/s 等）不能靠隐式约定 | S2（§7.2 顶层结构、§7.4 单位规则）、S3（缺口） | 设计文档字段清单 vs v1.0 表结构比对 | v1.1 含 units 表 + created_utc/polarization_channels/polarization_energy_reference meta 项且加载校验 | 单位仅文档性、外部无读方 | **pass** |
| D2 display_name / aspect 适用区间进表 | 数据库应承载可解释性（显示名）与完整适用条件（aspect 区间）；加载往返保真 | S2（§7.3 示例、§7.4 适用条件）、S8（结构体缺口） | 设计文档示例字段 vs 结构体字段比对 | 入表 + 加载进结构体 + 校验（min≤max）；Matcher 行为零变化 | 仅存 schema 不加载（往返不保真） | **pass（narrow：承载与校验，不消费）** |
| D3 语义分组表（模板组拆表） | 拍平 30 列不反映语义分组；模板组缺席用 NULL 表达不透明；加新模板组需 ALTER 大表 | S1 + 用户确认（语义分组表） | v1.0 表结构评审 | 4 模板组各一表，(model_id, profile_id) 复合主键 1:1 挂 profile；行存在=组存在 | 拍平仍可维护（无扩展证据） | **pass** |
| D4 DDL 权威资产 | 两处事实源（测试内联 DDL vs 加载器 SELECT）必然漂移——已有事故先例 | S4（迁移期 rcs 列名事故） | 实现后 grep：DDL 只存在于 schemas/ 单源 | `schemas/recognition/recognition_feature_database.sql` 为唯一源；测试经 configure_file 引用 | 单源不可行（构建侵入） | **pass** |
| D5 建库工具 | 仓库零 .db 文件、无交付途径；外部写库需手写 INSERT（30 列拍平 + 复合主键易错） | S5 + 用户确认（建库工具） | 工具产出的示例库可被加载器加载 | python3（stdlib）工具 JSON→DB；示例库文件提交并可加载 | 手写 SQL 即可（无外部交付方） | **pass** |
| D6 schema 演进机制 | `schema_version` 单字符串硬编码，无版本策略；未来 1.2/2.0 无规则 | S5、`RecognitionFeatureDatabase.cpp:27` | 版本校验代码评审 | 版本策略文档化（major 破坏/minor 增量）；本版精确匹配 "1.1" | 迁移脚本/版本范围逻辑（无存量库，YAGNI） | **narrow（仅策略文档化 + 拒绝旧版）** |
| D7 加载器与 DDL 一致性守护 | 加载器列名与权威 DDL 漂移可被全字段加载用例捕获 | S4 | 全字段 DB（所有模板组所有列填充）加载用例 | 用例由权威 DDL 构造且全绿 | 用例无法捕获（列名错位不报错） | **pass** |
| D8 units 语义强校验（rcs=="dBsm"） | 匹配数学是 dBsm 域；units.rcs 声明其他单位会静默破坏匹配语义——宁拒绝不静默（边界处理优先） | 设计文档 §7.4（"使用 dBsm，不允许混用平方米"）、用户边界处理指令 | 构造 units.rcs='m2' 库 → 加载必须失败 | 加载器拒绝 `units.rcs != "dBsm"` | 单位声明纯文档性（静默接受） | **pass** |
| D9 v1.0 兼容读取 | 需要读取存量 v1.0 文件 | 仓库零 .db 文件、无存量基线（S5）；v1.0 仅存在于测试构造 | `find . -name "*.db"` | — | 无存量即无需兼容层（延续既有"项目未上线、不做兼容"指令） | **reject（无兼容层）** |
| D10 aspect/display_name 立即参与匹配（Matcher 门控） | aspect 区间应立即作为 profile 适用性门控；display_name 应进入识别结果 | 无需求证据（设计文档首期适用条件为 min_snr/max_range） | — | — | 无消费需求；先承载后消费，需求出现时以新 freeze item 评估 | **reject（YAGNI）** |

Stage A 结论：**7 pass（含 2 narrow）/ 2 reject**，进入冻结契约。

## 4. 冻结契约

### Proven requirement

schema v1.0（JSON 拍平）演进为 v1.1：模板组拆为语义表、自描述元数据（units/created_utc/通道约定）入库、
display_name 与 aspect 适用区间入表并加载（不消费）；DDL 成为权威单源资产；提供建库工具与示例库文件；
运行期加载/失败/回滚/replay 契约与 Matcher/Tracker 行为零变化；`RecognitionFeatureDatabase` 对外接口不变。

### Allowed scope

- 模块/目录：
  - `src/airborne_radar/recognition/`（`RecognitionFeatureDatabase.{h,cpp}`：v1.1 加载 + 结构体扩展 display_name/aspect）
  - `schemas/recognition/recognition_feature_database.sql`（**新建**，权威 DDL 单源，与 `schemas/replay/` 同惯例）
  - `tools/recognition_db_builder.py`（**新建**，python3 stdlib：JSON → .db）
  - `examples/configs/recognition/`（**新建**：输入示例 JSON + 生成的 `target_feature_database_v1.1.db`）
  - `tests/`（`RecognitionSqliteTestUtil.h` 改引用权威 DDL；`ar_recognition_database_test.cpp` / `ar_recognition_integration_test.cpp` / `ar_recognition_scenario_test.cpp` 改 v1.1 构造；新增一致性/示例库用例）
  - `tests/CMakeLists.txt` 或分区 cmake（configure_file 生成 DDL 头，无公共构建系统改动）
- 类/函数：`RecognitionFeatureDatabase::Load`（签名不变，v1.1 实现）；文件内静态读取函数集（表分组化改造）
- 文档：`docs/airborne_radar/algorithms.md`（数据库行 + [evidence]）、`docs/airborne_radar/boundaries.md`（数据库契约：自描述/单位强校验/版本策略）、`remote_recognition_design(1)(1).md` §7.1/7.2（文件命名 .json → .db、顶层结构对齐 v1.1）、本规划 Stage C 回写

### Explicitly out of scope

- Public headers：`include/1q/` 零改动
- Matcher/Tracker：**行为零变化**（D10 reject；display_name/aspect 只承载不消费）
- Schema/trace/replay：fbs/codec 零改动
- 测试阈值/skip：不调整
- 兼容层：**禁止** v1.0 读取（D9 reject）、禁止 JSON 双路径
- 运行时连接：继续禁止在 `RecognitionFeatureDatabase` 中持有 SQLite 连接
- 迁移脚本/版本范围逻辑（D6 narrow：仅策略文档化）
- 跨模块通用数据库层（S7 无证据）

### Behavior boundary

- 输入：`database_path` 语义不变（v1.1 文件路径）；空串仍表示未配置
- 输出：`IsLoaded()/version()/database_id()/models()/categories()` 语义不变；结构体新增字段（display_name/aspect）不影响既有读取路径
- 错误/回退：打开失败、损坏文件、校验失败 → `Load` 返回 false + error（含路径与表/字段上下文），保持旧库；不抛异常
- 生命周期：加载期打开（只读）→ 读 meta/units/表 → 校验 → 关闭连接 → 填充 candidate → 原子提交；成功后无连接驻留
- 版本语义：`schema_version == "1.1"` 精确匹配（拒绝 "1.0"，D9）；`version()`/`database_id()` 来自 meta；新增 units/created_utc/通道约定必填校验（D1）

### Acceptance gates

- Build：`llvm-ninja-release-local` + `llvm-ninja-debug-local` 双 preset 全量构建
- Focused tests：`ar_recognition_database_test`（v1.1 失败用例一一对应 + 新增 units/版本/一致性用例）/ `ar_recognition_integration_test` / `ar_recognition_feature_test` / `ar_recognition_scenario_test` 全绿
- 示例库：`examples/configs/recognition/target_feature_database_v1.1.db` 加载用例全绿
- Contract tests：replay roundtrip（RecognitionFields / DefaultState / SessionConfig）零改动通过
- 全量：ctest 全绿（基线 53/53 不降；新用例为增量）

### Non-goals

- 运行时查询/增量更新/并发写/热替换（延续 F9 reject）
- v1.0 兼容层、迁移脚本（D9 reject）
- Matcher 适用性门控扩展、display_name 消费（D10 reject）
- 跨模块通用数据库层（S7）
- 性能基准（加载期一次性初始化）

## 5. 设计要点（Stage B 前定稿）

### 5.1 schema v1.1（权威 DDL 草案）

```sql
PRAGMA foreign_keys = ON;

-- 自描述元数据：键值表承载字符串元数据（schema_version='1.1' 精确匹配，
-- database_id/version/created_utc 必填；polarization_channels 逗号分隔，如 'H,V'）
CREATE TABLE meta(key TEXT PRIMARY KEY, value TEXT NOT NULL);

-- 量纲声明：7 个已知量必填（rcs/speed/altitude/acceleration/turn_radius/polarization/range），
-- 值非空；rcs 必须为 'dBsm'（D8 强校验：匹配数学是 dBsm 域）
CREATE TABLE units(
  quantity TEXT PRIMARY KEY,
  unit TEXT NOT NULL);

CREATE TABLE categories(
  category_id TEXT PRIMARY KEY,
  display_name TEXT,              -- 可空（文档性显示名，D2 承载不消费）
  prior REAL NOT NULL CHECK(prior > 0));

CREATE TABLE models(
  model_id TEXT PRIMARY KEY,
  category_id TEXT NOT NULL REFERENCES categories(category_id),
  display_name TEXT,
  prior REAL NOT NULL CHECK(prior > 0));

-- 适用条件：min_snr_db 可空（不约束）；max_range_resolution_m 可空/NULL（不限）；
-- aspect 区间可空 = 全范围（缺省 [-180,180]/[-90,90]），加载校验 min <= max
CREATE TABLE profiles(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL REFERENCES models(model_id),
  min_snr_db REAL,
  max_range_resolution_m REAL,
  aspect_az_min_deg REAL, aspect_az_max_deg REAL,
  aspect_el_min_deg REAL, aspect_el_max_deg REAL,
  PRIMARY KEY (model_id, profile_id));

-- 模板组表（D3）：行存在 = 组存在；mean 可空（缺省 0.0f）；std 必填且 > 0
-- 复合外键 (model_id, profile_id) REFERENCES profiles（SQLite 支持复合 FK）
CREATE TABLE rcs_templates(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL,
  mean_dbsm REAL,
  std_db REAL NOT NULL CHECK(std_db > 0),   -- 列名保持 std_db（错误信息/测试对齐）
  azimuth_variation_db REAL,
  elevation_variation_db REAL,
  minimum_aspect_coverage_deg REAL,
  PRIMARY KEY (model_id, profile_id),
  FOREIGN KEY (model_id, profile_id) REFERENCES profiles(model_id, profile_id));

CREATE TABLE motion_templates(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL,
  speed_mean REAL, speed_std REAL NOT NULL CHECK(speed_std > 0),
  altitude_mean REAL, altitude_std REAL NOT NULL CHECK(altitude_std > 0),
  acceleration_mean REAL, acceleration_std REAL NOT NULL CHECK(acceleration_std > 0),
  turn_radius_mean_log10 REAL, turn_radius_std_log10 REAL NOT NULL CHECK(turn_radius_std_log10 > 0),
  PRIMARY KEY (model_id, profile_id),
  FOREIGN KEY (model_id, profile_id) REFERENCES profiles(model_id, profile_id));

CREATE TABLE polarization_templates(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL,
  energy_difference_mean REAL, energy_difference_std REAL NOT NULL CHECK(energy_difference_std > 0),
  relative_difference_mean REAL, relative_difference_std REAL NOT NULL CHECK(relative_difference_std > 0),
  energy_sum_mean REAL, energy_sum_std REAL NOT NULL CHECK(energy_sum_std > 0),
  PRIMARY KEY (model_id, profile_id),
  FOREIGN KEY (model_id, profile_id) REFERENCES profiles(model_id, profile_id));

CREATE TABLE range_profile_templates(
  profile_id TEXT NOT NULL,
  model_id TEXT NOT NULL,
  length_mean REAL, length_std REAL NOT NULL CHECK(length_std > 0),
  peak_count_mean REAL, peak_count_std REAL NOT NULL CHECK(peak_count_std > 0),
  peak_energy_concentration_mean REAL,
  peak_energy_concentration_std REAL NOT NULL CHECK(peak_energy_concentration_std > 0),
  minimum_bandwidth_hz REAL,
  PRIMARY KEY (model_id, profile_id),
  FOREIGN KEY (model_id, profile_id) REFERENCES profiles(model_id, profile_id));
```

要点：
- **组存在语义**：v1.0 用 30 列 NULL 表达模板缺席 → v1.1 用模板表行存在表达；加载器按行存在判定 `present`，
  语义与 v1.0 完全对齐（`ReadTemplate` 的 `present` 输出 → 表行存在判定 + 列非空校验）
- **std 必填校验保持**：`std > 0` 由 CHECK 兜底 + 加载器显式校验（错误信息含表/列，S10 保持）
- **复合主键/外键**：`(model_id, profile_id)` 保持"模型内唯一"语义（v1.0 决策延续）；模板表 1:1 挂 profile
- **units 表**：独立表而非 meta 键（可枚举校验 7 个已知量；meta 保持自由键值）
- **CHECK 语义注意**：SQLite CHECK 对 NULL 结果为 NULL 即通过——故 std 列必须 `NOT NULL + CHECK(std>0)` 组合

### 5.2 加载流程（v1.1）

```
open(SQLITE_OPEN_READONLY) → PRAGMA foreign_keys → 读 meta（schema_version=="1.1"、
database_id/version/created_utc/polarization_channels/polarization_energy_reference 必填非空）
→ 读 units（7 已知量必填非空；units.rcs=="dBsm" 强校验 D8）
→ 读 categories（display_name 可空）→ 读 models（display_name 可空、category 引用校验）
→ 读 profiles（aspect min<=max 校验、model 引用校验、按 model 分组）
→ 读 4 个模板表（行存在=组存在；std>0；mean 可空缺省 0）
→ 显式校验（std>0/prior>0/引用/复合主键唯一由约束保证）
→ 关闭连接 → 填充 candidate → loaded_=true 原子提交
失败任意一步 → error 含路径+上下文，返回 false，调用方保持旧库
```

结构体扩展（D2 narrow，`RecognitionFeatureDatabase.h` 内部头）：
- `RecognitionCategoryEntry` / `RecognitionModel`：+ `std::string display_name{}`
- `RecognitionModelProfile`：+ `float aspect_az_min_deg{-180.0f} / aspect_az_max_deg{180.0f} / aspect_el_min_deg{-90.0f} / aspect_el_max_deg{90.0f}`（NULL → 全范围缺省）
- Matcher/Tracker 零改动（不消费新字段，D10 reject 的边界）

### 5.3 DDL 权威资产（D4）

- 单源：`schemas/recognition/recognition_feature_database.sql`（与 `schemas/replay/` 目录惯例一致）
- C++ 测试引用：`configure_file` 把 .sql 生成为头文件（字符串常量），测试 util 改为引用（消除内联副本）；
  构建产物仅测试分区可见（加 `INCLUDE_DIRS`，不进入公共构建面）
- 建库工具直接读同一 .sql 文件（`sqlite3.executescript`）——C++ 加载器、C++ 测试、python 工具三方共用同一 DDL 源
- 一致性守护（D7）：全字段用例——权威 DDL 构造、所有模板组所有列填充的库，加载必须成功且字段值逐一断言；
  任一 SELECT 列名漂移即用例失败

### 5.4 建库工具（D5）

- 落点：`tools/recognition_db_builder.py`（python3 stdlib：json/sqlite3/argparse/datetime；S9 有 python 先例）
- 输入：设计文档 §7.3 格式 JSON（含 units/display_name/created_utc/aspect 区间）
- 输出：schema 校验通过的 .db 文件（内部复用加载器同规则：std>0、prior>0、引用存在、aspect min<=max）
- 输入示例：`examples/configs/recognition/recognition_database_input.json`（设计文档两个占位型号示例）
- 生成物：`examples/configs/recognition/target_feature_database_v1.1.db`（提交入库，可再生成；命令写入工具 docstring）
- 验收：示例库被加载器加载用例全绿（集成层）

### 5.5 版本策略（D6 narrow）

- `schema_version` 语义：`major.minor` 字符串；**major 变更 = 破坏性**（加载器拒绝，需显式 freeze item）；
  **minor 变更 = 增量**（新增可空表/列，加载器需同步读取，仍精确匹配自身版本）
- 本版：精确匹配 `"1.1"`；拒绝 `"1.0"`（D9：无存量库，不做兼容层）
- 策略写入 `docs/airborne_radar/boundaries.md`（变更规则），不实现版本范围逻辑（无 1.2 需求，YAGNI）

## 6. Stage B 实现计划（按提交切分）

**C1 DDL 权威资产落地（独立提交，行为零变化）**
1. `schemas/recognition/recognition_feature_database.sql`（v1.0 结构先落地为单源）
2. `tests/CMakeLists.txt`（或分区 cmake）：configure_file 生成 DDL 头；测试 util 改引用；新增全字段一致性用例
3. 验证：双 preset 构建 + 识别测试全绿（schema 未变，纯资产迁移）

**C2 schema v1.1（核心提交，含测试更新）**
4. `.sql` 更新为 v1.1（5.1 DDL）
5. `RecognitionFeatureDatabase.{h,cpp}`：v1.1 加载（meta 新键 + units 表 + 模板表行判定 + aspect/display_name）；结构体扩展
6. 测试改造：`RecognitionSqliteTestUtil.h`（v1.1 构造）/ `ar_recognition_database_test.cpp`（失败用例：缺表、std==0、
   units.rcs!='dBsm'、缺 units 量、aspect min>max、schema 9.9、缺 created_utc）/ `ar_recognition_integration_test.cpp` /
   `ar_recognition_scenario_test.cpp`
7. 验证：双 preset 构建 + 识别/replay 测试全绿

**C3 建库工具 + 示例库**
8. `tools/recognition_db_builder.py` + 输入示例 JSON
9. 生成并提交 `examples/configs/recognition/target_feature_database_v1.1.db`
10. 新增示例库加载用例（集成层）
11. 验证：工具命令行运行 + 加载用例全绿

**C4 文档回写**
12. `docs/airborne_radar/algorithms.md`（数据库行：schema v1.1 + [evidence]）
13. `docs/airborne_radar/boundaries.md`（数据库契约：自描述/units 强校验/版本策略/承载不消费边界）
14. `remote_recognition_design(1)(1).md` §7.1 文件命名 `.json` → `.db`、§7.2 顶层结构对齐 v1.1
15. 本规划 Stage C 回写

## 7. 验收门（Stage C）

- 双 preset 构建 + ctest 全量（基线 53/53 不降，新用例增量）
- replay 测试零改动通过（fbs/codec 未动）
- grep 确认：DDL 只存在于 `schemas/recognition/recognition_feature_database.sql` 单源（测试内联副本清零）；
  无 v1.0 兼容层残留
- 建库工具端到端：示例 JSON → .db → 加载用例全绿

## 8. 非目标与后续观察（reject/narrow 汇总）

- **D9 reject**：v1.0 兼容读取——仓库零存量库，无兼容层（延续"项目未上线、边界处理优先"指令）
- **D10 reject**：aspect/display_name 立即消费——无需求证据；先承载后消费，需求出现时以新 freeze item 评估
- **D6 narrow**：仅版本策略文档化；迁移脚本/版本范围逻辑待真实 1.2 需求再评估
- **后续观察项**：若 aspect 区间出现消费需求（profile 适用性门控），以新 freeze item 评估 Matcher 扩展

## 9. 待确认决策点（实现前）

1. **units 存储**：独立 `units` 表（推荐，可枚举校验 7 已知量）vs meta 键（`units.rcs` 等）
2. **units.rcs 强校验**：拒绝 `!= "dBsm"`（推荐，D8：匹配数学域；宁拒绝不静默）
3. **created_utc/通道约定必填**：v1.1 自描述契约必填非空（推荐）vs 可空（宽松）
4. **display_name/aspect 加载进结构体**：往返保真（推荐，D2 narrow）——结构体扩展但 Matcher 不消费
5. **aspect NULL 语义**：NULL = 全范围缺省 [-180,180]/[-90,90]（推荐，对应设计文档缺省区间）
6. **schema_version**：精确匹配 "1.1"、拒绝 "1.0"（推荐，D9 无存量）
7. **DDL→头生成**：`configure_file`（推荐，CMake 标准；仓库无先例但无侵入）vs 测试内联保留（否决，S4 两处事实源）
8. **建库工具语言**：python3 stdlib（推荐，S9 先例；C++ 工具需新可执行目标 + 参数解析，重）vs C++ 工具
9. **示例库二进制提交**：提交生成物（推荐，可再生成）——测试直接加载，不依赖运行环境有 python

## 10. Stage C Result（2026-08-04，实现后回写）

### Implemented scope

**C1 DDL 权威资产落地（c9b55601，独立提交，行为零变化）**
- `schemas/recognition/recognition_feature_database.sql`（v1.0 结构单源落地）
- `tests/CMakeLists.txt`：`configure_file` 生成 `recognition_feature_database_schema.h`；
  `CMAKE_CONFIGURE_DEPENDS` 追踪 .sql（变更自动 reconfigure，防生成头陈旧）
- `RecognitionSqliteTestUtil.h` 内联 DDL 移除，改引用生成头；airborne_radar unit/integration
  分区 INCLUDE_DIRS 加生成目录
- 验证：双 preset 构建 + release 53/53（schema 字节等价）

**C2 schema v1.1 核心迁移（02b4ec32）**
- `.sql` → v1.1：模板组四表（复合主键/复合外键挂 profiles）、units 表、meta 六键、
  categories/models 增 display_name（可空）、prior 收紧 `NOT NULL CHECK(prior>0)`、
  profiles 增 aspect 四列（NULL = 全范围）
- 加载器：meta 六键必填校验；units 七量纲必填 + `rcs=='dBsm'` 强校验；模板表行存在 =
  组存在（std 必填 > 0，mean 可空缺省 0）；aspect min<=max 校验；display_name 加载
- 结构体扩展：CategoryEntry/Model + display_name；Profile + aspect 四字段（缺省全范围）；
  Matcher/Tracker 零改动（D10 边界）
- **修复 v1.0 潜伏 bug**：turn_radius 模板列名（`motion_turn_radius_*_log10`）与读取器
  拼接名（`motion_turn_radius_mean`）不匹配 → 模板从未加载（恒 mean=0/std=1）。
  v1.1 列名直接匹配，模板数据往返保真（新增断言锁定）
- 测试：v1.1 构造重写；失败用例 9 个（缺表/零 std/单位错误/缺量纲/aspect 越界/未知
  引用/缺元数据/坏 schema 版本）；新增自描述字段与模板往返断言
- 验证：release 53/53；debug airborne_radar/replay 全绿

**C3 建库工具 + 示例库（d7bda3d3）**
- `tools/recognition_db_builder.py`：python3 stdlib，JSON → SQLite；DDL 读权威单源；
  校验规则与加载器一致；负例（错误单位/aspect 越界/std 0）拒绝且退出码 1
- `examples/configs/recognition/`：输入示例 JSON（设计文档 §7.3 数据）+ 生成的
  `target_feature_database_v1.1.db`（提交入库）
- 新增集成用例 `ar_recognition_example_database_test.cpp`：加载提交的示例库，
  断言自描述字段与模板数据往返保真（Integration.cmake 注入路径宏）
- 验证：release 53/53

**C4 文档回写（本提交）**
- `algorithms.md`：数据库行更新（schema v1.1 + 双证据）
- `boundaries.md`：F1 措辞（v1.1 自描述 meta 键）+ 新增"识别特征数据库契约"小节
  （自描述/DDL 单源/加载期读取器/承载不消费/版本策略）
- 设计文档 §7.1/7.2：文件命名 `.json` → `.db`、顶层结构对齐 v1.1（表映射 + 工具指引）

### Validation

- `cmake --preset llvm-ninja-release-local` + build：pass
- `ctest --preset llvm-ninja-release-local -j 4`：**53/53 pass**（含示例库加载用例）
- `cmake --preset llvm-ninja-debug-local` + build：pass；识别/replay 测试全绿
- `python3 tools/recognition_db_builder.py`：正例生成成功；负例（`units.rcs='m2'`、
  aspect min>max、`std_db=0`）全部拒绝，错误消息含字段路径，退出码 1
- grep：DDL 单源（`schemas/recognition/` 仅一份 .sql）；无 v1.0 兼容层残留

### Residual risks

- **debug 下 `integration::airborne_radar` 偶发失败（观察项）**：C1/C3 验证期间各出现一次
  （首跑失败、复跑通过；release 恒绿）。形如 scenario 测试在 debug + 并行（-j 4，与
  69s 的 performance::cross_domain 同跑）下的预算边缘波动，非本次改动回归（C1 前基线
  亦有同源先例）。后续若频繁复现，以新 freeze item 评估 scenario 预算/串行化。
- **turn_radius 模板行为变化**：v1.0 潜伏 bug 修复使该模板从"恒缺省 {0,1}"变为真实数据。
  直线目标（turn_radius 无效）不受影响；弯航目标在 v1.0 下实际用的是缺省模板，
  v1.1 起用真实模板——属修复而非兼容破坏（项目未上线、无存量库）。
- **vendored 路径未本机验证**：`PACKAGE_MANAGER=none`（Windows 脚手架）的 SQLite 段
  与 conan provider 同契约但未真实构建（延续 v1.0 迁移残留风险）。
- **建库工具为脚本而非测试覆盖**：正例/负例验证为手工运行；如需纳入 CI，可增加
  python 冒烟测试（当前无 pytest 基础设施，未引入）。

### Follow-up freeze items

- debug 偶发 integration 失败观察项（见上）——复现频次升高时再评估
- 真实基线进入万级型号规模时，重新评估"加载期全量读入 vs 运行时连接/按需查询"（延续 F9）
- aspect 区间出现消费需求（profile 适用性门控）时，以新 freeze item 评估 Matcher 扩展（D10 再进入条件）
