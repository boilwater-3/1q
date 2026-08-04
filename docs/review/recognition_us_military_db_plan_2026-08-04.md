# 识别特征数据库美方型号扩展与场景验证规划 (US military types + scenarios)

Status: draft
Date: 2026-08-04
Upstream: `remote_recognition_design(1)(1).md`（§1 范围、§7 数据库）、`recognition_database_v11_design_plan_2026-08-04.md`（schema v1.1）
Flow: 数据扩展 + 场景验证（schema/算法零改动，不走 freeze 契约；断言阈值以实测校准）

## 1. 背景与目标

原始需求要求识别**战斗机、轰炸机、导弹、无人机等常见美方型号**，当前示例库仅含
`BALLISTIC_EXAMPLE_A`/`NEAR_SPACE_EXAMPLE_A` 两个占位型号。本规划：

1. 扩展 `examples/configs/recognition/recognition_database_input.json`：新增
   FIGHTER/BOMBER/MISSILE/UAV 四类共 15 个常见美方型号（保留现有 2 个），经
   `tools/recognition_db_builder.py` 重建并交付 `target_feature_database_v1.1.db`。
2. 新建多场景集成测试，端到端加载**交付库**验证识别逻辑正确性：命中型号/类别、
   置信度、margin、同类歧义与跨类隔离边界。
3. 数据为公开渠道估算值（非敏感占位参数），沿用设计文档 §7.3 声明，不作真实情报数据。

## 2. 型号集（公开估算，非敏感占位参数）

| 类别 | 型号 | RCS (dBsm) | 巡航速度 (m/s) | 巡航高度 (m) | 机长 (m) | 备注 |
|---|---|---|---|---|---|---|
| FIGHTER | F-16C | 0.8 | 250 | 10500 | 15.1 | RCS 1.2 m²（GlobalSecurity 表） |
| FIGHTER | F-15E | 11.8 | 265 | 12000 | 19.4 | RCS 10-25 m² 取中值 |
| FIGHTER | F/A-18E | -10.0 | 250 | 10500 | 18.3 | RCS "0.1 class"（GS 表） |
| FIGHTER | F-22A | -37.0 | 520 | 16000 | 18.9 | 超巡剖面，std 加大（双模态） |
| FIGHTER | F-35A | -27.0 | 255 | 12000 | 15.7 | RCS 0.0015-0.005 m² |
| BOMBER | B-52H | 20.0 | 240 | 10000 | 48.5 | 教科书级 ~100 m² |
| BOMBER | B-1B | 3.8 | 270 | 100（低空） | 44.5 | 低空突防剖面，RCS≈B-52 的 1/50 |
| BOMBER | B-2A | -10.0 | 250 | 13000 | 21.0 | Wikipedia 引用 ~0.1 m² |
| MISSILE | BGM-109 | -10.0 | 255 | 40（掠海） | 5.6 | Mach 0.74，30-50 m AGL |
| MISSILE | AGM-158A | -25.0 | 240 | 80（低空） | 4.3 | 速度/高度无官方值，取代表值 |
| MISSILE | AGM-86C | -5.0 | 246 | 40（低空） | 6.3 | 非隐身小弹 |
| UAV | MQ-9A | -12.0 | 78 | 7600 | 11.0 | 巡航速度来源冲突，取 150-170 kn |
| UAV | RQ-4B | -5.0 | 159 | 18000 | 14.5 | 大雷达罩，RCS 估 -9~-1 |
| UAV | MQ-4C | -6.0 | 160 | 16500 | 14.5 | 巡航速度推导（低置信） |
| UAV | MQ-1C | -15.0 | 60 | 4800 | 8.5 | 最小机身，巡航推导 |

来源：Wikipedia/GlobalSecurity/USAF 公开事实表（af.mil 直接抓取被 403，经 Wikipedia
引用转述）；RCS 全部无官方值，为公开估算区间中值；加速度/转弯半径/距离像峰数为
仿真调参占位（v²/(g·n) 推导）。速度置信度高、高度中、RCS 低。

数据边界：model prior 统一 1.0（best_score≈相似度，保证 ≥0.6 可确认；置信度由特征分离度
自然决定）；新条目 gate 字段（`minimum_aspect_coverage_deg`/`minimum_bandwidth_hz`/
`max_range_resolution_m`）置 NULL 防 gating 误伤；`min_snr_db` 6.0；meta `version`
1.0.0 → **1.1.0**（内容扩充 minor 语义）。

## 3. 场景矩阵（交付库端到端）

| # | 场景 | 签名（目标真值） | 期望断言 | 实测（校准） |
|---|---|---|---|---|
| S1 | 战斗机识别 | F-16C（250 m/s、10500 m、0.8 dBsm） | kModelConfirmed、F-16C、conf≥0.15 | best 0.79、margin 0.37、conf 0.20 |
| S2 | 轰炸机识别 | B-52H（240、10000、20） | kModelConfirmed、B-52H、conf≥0.20 | best 0.82、margin 0.43、conf 0.26 |
| S3 | 巡航导弹识别 | BGM-109（255、40、-10） | kModelConfirmed、BGM-109、conf≥0.15 | best 0.80、margin 0.20、conf 0.18 |
| S4 | 无人机识别 | MQ-9A（78、7600、-12） | kModelConfirmed、MQ-9A、conf≥0.25 | best 0.82、margin 0.46、conf 0.33 |
| S5 | 同类歧义边界 | AGM-86C（246、40、-5） | 类别 MISSILE 始终正确；型号确认；conf≥0.15 | best 0.83、margin 0.34、conf 0.20 |
| S6 | 跨类隔离 | 同帧 F-16C + BGM-109 | category/model_accuracy == 1.0、双 kModelConfirmed | 100% |
| S7 | 置信度排序 | MQ-9A vs AGM-86C | conf(MQ-9A) > conf(AGM-86C)+0.08 | 0.33 vs 0.20 |

每场景 5 cycles，聚合断言沿用现有模式（确认率≥80%、型号正确率≥70%、类别正确率 100%）。

## 4. 实施步骤

- **S1 调研**：3 个并行子代理（战斗机 / 轰炸机+导弹 / 无人机）按 JSON 字段模板产出
  公开渠道参数表（含来源与置信度标注）。
- **S2 数据写入**：扩展 JSON（meta version → 1.1.0、4 新类别、15 新型号）；重建 DB。
- **S3 测试**：更新 `ar_recognition_example_database_test.cpp`（categories==6、models==17、
  version==1.1.0、新条目抽查）；新建 `ar_recognition_us_military_scenario_test.cpp`
  （加载交付库，校准阈值）；公共枚举加性扩展（kFighter/kBomber/kMissile/kUav +
  CategoryToPublic 映射 + replay codec 白名单）。
- **S4 文档**：设计文档 §1 范围 + §7.3 型号表 + §11.1 枚举；boundaries 类别映射；
  algorithms 证据链；本规划 Stage C 回写。
- **S5 验收**：builder 负例、双 preset 构建、聚焦 + 全量 ctest、grep 检查。

## 5. Stage C Result（2026-08-04，实现后回写）

### Implemented scope

- JSON：6 类别（新增 FIGHTER/BOMBER/MISSILE/UAV）、17 型号（15 新 + 2 既有，既有条目零改动）；
  meta `version` 1.0.0 → 1.1.0。
- DB：重建 `target_feature_database_v1.1.db`（6 类别、17 型号、模板表各 17 行）。
- 公共 API：`ArRecognitionCategory` 加性扩展 kFighter=4/kBomber=5/kMissile=6/kUav=7
  （fbs 存 int，schema 零改动）；`CategoryToPublic` 映射 + replay codec 白名单
  `<= kUav`；既有值 0-3 字节兼容。
- 测试：`ar_recognition_example_database_test.cpp` 更新（6/17/1.1.0 + 新条目抽查）；
  新建 `ar_recognition_us_military_scenario_test.cpp`（7 场景，加载交付库）。
- 文档：design §1 范围/§1b SQLite/§7.3 型号表/§11.1 枚举；boundaries 类别映射契约；
  algorithms 证据链。

### 场景构造关键发现（管线帧约定）

识别管线在**平台 ENU 局部系**观测目标：`altitude = 平台海拔 + sin(lat)·Δz_ecef +
cos(lat)cos(lon)·Δx`。场景构造对策（已固化进测试 `MakeInput`/`AltitudeOffsetFor`）：

1. 平台经度取 **90°**（cos=0）：目标沿 x 的运动不再污染高度观测（lon=121° 时每周期
   漂移 ~55 m，且 sin 补偿被抵消）。
2. 高度偏移 = (模板高度 − 平台海拔)/sin(31°)（ECEF z 差 → ENU 上向投影）。

其余事实：场景 SNR ≈ 30 dB（RCS 质量因子实际由视角覆盖 10°/60 限制，非 SNR）；匀速
目标加速度观测 ≈ 0 → 运动相似度 ≈ (1+1+0.004~0.17)/3，加速度子特征不匹配是场景
特性而非缺陷（模板保留真实机动量级，对机动目标仍有效）。

### Validation

- release 聚焦：integration 14/14（7 新场景 + 7 既有场景/示例库）、unit 36/36 全绿。
- 校准阈值留 ~30% 裕量（S4 0.25 vs 实测 0.33；S7 0.08 vs 实测 0.13）。
- 双 preset 全量 + grep 检查见规划验收段（Stage 5 完成后回写）。

### Residual risks

- **加速度子特征未在场景中命中**（匀速目标）：运动相似度上限 ~0.72、best_score ~0.8；
  若未来场景要求机动目标验证，需给目标加加速度剖面（v(t)=v0+a·t）或新 freeze item。
- **置信度被 17 型号稀释**（conf 0.18-0.33）：为库规模的自然结果，S7 用相对断言规避。
- **数据性质**：RCS/高度等为公开估算中值，置信度标注见 §2；不作为真实情报数据。
- **枚举扩展**：加性（0-3 不变）字节兼容；若未来重排枚举值将破坏 replay。
