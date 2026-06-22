# SAR 工作目录结构重组方案

> **状态**: ✅ 已执行（2026-06-22，149 份文档归一完成，零代码改动）
> **创建**: 2026-06-22
> **范围**: 全仓非代码 markdown 文档（~150 份）的目录归一与命名归一
> **决策依据**: 顶层 SAR 文档与 `docs/` 双地分散、核心设计被 CONTRACT 淹没、planning 工作文件污染顶层、命名大小写不统一

---

## 1. 背景与目标

当前工作空间非代码 markdown 文档约 **150 份**，分散在仓库顶层与 `docs/` 两处，且同类文档（合约 / 验收 / 决策）被拆分到不同位置，核心设计文档辨识度低。本方案在不改动任何代码、不破坏文档间引用的前提下，将全部文档按**生命周期类型**归一到 `docs/` 子树，统一命名风格，并隔离临时工作文件。

**目标**:

1. 顶层仅保留项目元数据（`README.md` / `CLAUDE.md` / `AGENTS.md`）。
2. 全部 SAR 文档归一到 `docs/sar/`，按类型分目录。
3. 跨模块公共手册、模块评审、工作日志各自独立子目录。
4. 命名统一为小写 snake_case，去除冗余前缀与类型后缀（类型由目录表达）。
5. 文档间引用（极稀疏，仅 5 处）同步更新。

---

## 2. 现状盘点

### 2.1 顶层（52 份非代码 md）

| 类别 | 数量 | 说明 |
|---|---|---|
| 项目元数据 | 3 | `README.md` / `CLAUDE.md` / `AGENTS.md`（保留顶层） |
| 子能力合约 `SAR_*_CONTRACT.md` | 43 | 权威契约边界 |
| 核心设计 | 4 | `SAR_MODULE_DESIGN.md`、`sar_construction_scheme_complete.md`、`SAR_PHASE1_ENGINEERING_CONTRACT.md`、`SAR_PHASE2_REFERENCE_IMAGING_CONTRACT.md` |
| 错位决策 | 1 | `SAR_OMEGA_K_RELATIVE_DELAY_TRANSFORM_FOLLOWUP_DECISION.md`（本属 decisions） |
| planning 工作文件 | 3 | `task_plan.md`(2493) / `progress.md`(2372) / `findings.md`(1312) |
| 非 SAR | 1 | `jsbsim_aircraft_analysis.md` |

### 2.2 `docs/`（98 份）

| 类别 | 数量 | 现位置 |
|---|---|---|
| `*_acceptance_report.md` | 51 | `docs/` 根 |
| `*_followup_decision.md` | 27 | `docs/` 根 |
| `*_decision.md`（纯） | 4 | `docs/` 根 |
| `*_audit.md` | 4 | `docs/` 根 |
| `*_report.md`（纯，非 acceptance） | 4 | `docs/` 根 |
| `*_research.md` | 1 | `docs/` 根 |
| 错位合约 | 1 | `docs/sar_public_focused_image_output_contract.md` |
| 其它 | 1 | `docs/sar_next_incomplete_capability_selection.md` |
| 跨模块公共手册 `public_*_manual.md` | 3 | `docs/` 根 |
| 模块评审 | 2 | `docs/review/` |

### 2.3 核心问题

1. **同类两地分散**:合约在顶层、验收/决策在 `docs/`，同一子能力（如 `omega_k_stolt_interpolation`）的三类文档被拆到两处。
2. **核心设计被淹没**:4 份核心设计混在 43 份 CONTRACT 中。
3. **工作文件污染顶层**:三份 planning 产物合计 6000+ 行与正式文档同级。
4. **命名不统一**:顶层 `SAR_` 大写 vs `docs/` `sar_` 小写；后缀 `CONTRACT` 大写。
5. **`docs/` 语义退化**:96/99 为 `sar_` 前缀。

### 2.4 有利条件

- 文档间**交叉引用极稀疏**:全仓 .md 互引最高仅 2 次，核心三份设计文档构成自洽引用三角。移动几乎不破坏链接。
- 绝大部分文档已纳入 git 跟踪（顶层 46、`docs/` 94），`git mv` 可保留历史。

---

## 3. 重组原则

1. **目录表达类型，文件名表达子能力**:跨目录同名 stem 是预期且有益的（一眼看出某子能力的合约 / 验收 / 决策三件套）。
2. **核心设计独立突出**:`docs/sar/design/` 仅放 4 份权威入口。
3. **临时工作文件隔离**:planning 产物移入 `docs/worklog/`，不与正式文档混级。
4. **非 SAR 文档独立**:公共手册、模块评审、JSBSim 分析各自归位，不进 `docs/sar/`。
5. **不改代码、不改文档内容**:本次仅移动 + 重命名 + 更新 5 处引用;文档正文不动。

---

## 4. 命名归一规则

| 规则 | 说明 | 示例 |
|---|---|---|
| 去前缀 | 去掉 `SAR_` / `sar_`（SAR 上下文由 `docs/sar/` 隐含） | `SAR_OMEGA_K_*` → `omega_k_*` |
| 去类型后缀 | 类型由目录表达，文件名去掉 `_CONTRACT` / `_acceptance_report` / `_followup_decision` / `_decision` / `_audit` / `_research` / `_report` | `*_acceptance_report.md` → `acceptance/*.md` |
| 全小写 | snake_case | `SAR_L3_BP_CONTRACT.md` → `l3_bp.md` |
| 核心设计特例 | 4 份单独命名，见映射表 design 段 | `SAR_MODULE_DESIGN.md` → `module_design.md` |
| public 手册 | 去 `public_` 前缀（由 `docs/public/` 隐含） | `public_model_config_manual.md` → `model_config_manual.md` |
| review | 去 `sar_` 前缀与 `_review` 后缀 | `sar_module_algorithm_capability_review.md` → `module_algorithm_capability.md` |
| worklog | 保留原名（planning skill 产物） | `task_plan.md` → `task_plan.md` |

> **语义保留例外**:`SAR_OMEGA_K_POINT_TARGET_IMAGE_ACCEPTANCE_CONTRACT.md` 中段的 `ACCEPTANCE` 是子能力语义的一部分（非类型后缀），保留为 `omega_k_point_target_image_acceptance.md`。

---

## 5. 目标目录树

```text
仓库根/
├── README.md                      项目入口（保留）
├── CLAUDE.md                      工程约束（保留）
├── AGENTS.md                      代理约束（保留）
└── docs/
    ├── README.md                  ★ 新增:文档区总索引与导航
    ├── directory_restructure_plan.md   本方案（重组记录，重组后保留）
    ├── public/                    跨模块公共 API 手册(3)
    ├── review/                    模块评审(2，原位)
    ├── reference/                 非 SAR 跨模块参考(1，jsbsim)
    ├── worklog/                   planning 工作文件(3)
    └── sar/                       ★ SAR 文档归一(141)
        ├── README.md              ★ 新增:SAR 文档总索引（类型说明 + 三件套导航）
        ├── design/                核心设计(4)
        ├── contracts/             子能力合约(44)
        ├── acceptance/            验收报告(51)
        ├── decisions/             决策记录(32)
        └── audits/                审计 / 研究 / 基线 / 收尾(10)
```

**容量汇总**:design(4) + contracts(44) + acceptance(51) + decisions(32) + audits(10) = 141 份入 `docs/sar/`;public(3) + review(2) + reference(1) + worklog(3) = 9 份入其它 `docs/` 子目录。合计 150 份。

---

## 6. 完整 src → dst 映射表

### 6.1 `docs/sar/design/`（4 份核心设计）

| 源（顶层） | 目标 |
|---|---|
| `SAR_MODULE_DESIGN.md` | `docs/sar/design/module_design.md` |
| `sar_construction_scheme_complete.md` | `docs/sar/design/construction_scheme.md` |
| `SAR_PHASE1_ENGINEERING_CONTRACT.md` | `docs/sar/design/phase1_engineering.md` |
| `SAR_PHASE2_REFERENCE_IMAGING_CONTRACT.md` | `docs/sar/design/phase2_reference_imaging.md` |

### 6.2 `docs/sar/contracts/`（44 份）

| 源 | 目标 stem |
|---|---|
| `SAR_AUTOFOCUS_PHASE_ERROR_REFERENCE_CONTRACT.md` | `autofocus_phase_error_reference.md` |
| `SAR_CSA_MATH_REFERENCE_CONTRACT.md` | `csa_math_reference.md` |
| `SAR_CSA_PHASE_FUNCTION_INTERMEDIATE_TRUTH_CONTRACT.md` | `csa_phase_function_intermediate_truth.md` |
| `SAR_DETERMINISTIC_DISTRIBUTED_CLUTTER_CONTRACT.md` | `deterministic_distributed_clutter.md` |
| `SAR_EXPLICIT_CALIBRATION_OBSERVATION_CONTRACT.md` | `explicit_calibration_observation.md` |
| `SAR_EXTENDED_VARIABLE_PRF_RESAMPLING_QUALITY_MATRIX_CONTRACT.md` | `extended_variable_prf_resampling_quality_matrix.md` |
| `SAR_EXTERNAL_RAW_IQ_INPUT_CONTRACT.md` | `external_raw_iq_input.md` |
| `SAR_EXTERNAL_RAW_IQ_L2_MOTION_COMPENSATION_CONTRACT.md` | `external_raw_iq_l2_motion_compensation.md` |
| `SAR_EXTERNAL_RAW_IQ_PULSE_TRAJECTORY_CONTRACT.md` | `external_raw_iq_pulse_trajectory.md` |
| `SAR_IMAGE_QUALITY_COMPLETION_CONTRACT.md` | `image_quality_completion.md` |
| `SAR_INTERNAL_FOCUSING_SELECTOR_CONTRACT.md` | `internal_focusing_selector.md` |
| `SAR_INTERNAL_SESSION_CALIBRATION_REQUEST_CONTRACT.md` | `internal_session_calibration_request.md` |
| `SAR_INTERNAL_SLOW_TIME_RESAMPLING_REQUEST_CONTRACT.md` | `internal_slow_time_resampling_request.md` |
| `SAR_L2_MOTION_COMPENSATION_CONTRACT.md` | `l2_motion_compensation.md` |
| `SAR_L2_SESSION_INTEGRATION_CONTRACT.md` | `l2_session_integration.md` |
| `SAR_L3_BP_CONTRACT.md` | `l3_bp.md` |
| `SAR_L3_BP_SESSION_INTEGRATION_CONTRACT.md` | `l3_bp_session_integration.md` |
| `SAR_L3_WAYPOINT_TRAJECTORY_CONTRACT.md` | `l3_waypoint_trajectory.md` |
| `SAR_MISSING_PULSE_GAP_DIAGNOSTICS_CONTRACT.md` | `missing_pulse_gap_diagnostics.md` |
| `SAR_MISSING_PULSE_REJECTION_MATRIX_CONTRACT.md` | `missing_pulse_rejection_matrix.md` |
| `SAR_OMEGA_K_COMMON_STOLT_SUPPORT_CONTRACT.md` | `omega_k_common_stolt_support.md` |
| `SAR_OMEGA_K_COMPLEX_STOLT_INTERPOLATION_CONTRACT.md` | `omega_k_complex_stolt_interpolation.md` |
| `SAR_OMEGA_K_EXPLICIT_GRID_REDUCTION_CONTRACT.md` | `omega_k_explicit_grid_reduction.md` |
| `SAR_OMEGA_K_INDEPENDENT_PHYSICAL_TRUTH_INGESTION_CONTRACT.md` | `omega_k_independent_physical_truth_ingestion.md` |
| `SAR_OMEGA_K_MATH_REFERENCE_CONTRACT.md` | `omega_k_math_reference.md` |
| `SAR_OMEGA_K_POINT_TARGET_IMAGE_ACCEPTANCE_CONTRACT.md` | `omega_k_point_target_image_acceptance.md` |
| `SAR_OMEGA_K_REDUCED_RANGE_AXIS_CONTRACT.md` | `omega_k_reduced_range_axis.md` |
| `SAR_OMEGA_K_REFERENCE_PHASE_ABSOLUTE_RANGE_CONTRACT.md` | `omega_k_reference_phase_absolute_range.md` |
| `SAR_OMEGA_K_RELATIVE_DELAY_TRANSFORM_CONTRACT.md` | `omega_k_relative_delay_transform.md` |
| `SAR_OMEGA_K_TRUTH_PAYLOAD_DIGEST_VERIFICATION_CONTRACT.md` | `omega_k_truth_payload_digest_verification.md` |
| `SAR_PGA_GRADIENT_ESTIMATOR_TRUTH_COMPARISON_CONTRACT.md` | `pga_gradient_estimator_truth_comparison.md` |
| `SAR_PGA_SUPPORT_GRADIENT_TRUTH_CONTRACT.md` | `pga_support_gradient_truth.md` |
| `SAR_PHASE_REFERENCE_CONTRACT.md` | `phase_reference.md` |
| `SAR_RADIOMETRIC_CALIBRATION_CONTRACT.md` | `radiometric_calibration.md` |
| `SAR_RAW_HISTORY_SLOW_TIME_RESAMPLING_CONTRACT.md` | `raw_history_slow_time_resampling.md` |
| `SAR_RDA_APERTURE_PHASE_SPAN_DIAGNOSTIC_CONTRACT.md` | `rda_aperture_phase_span_diagnostic.md` |
| `SAR_RDA_AZIMUTH_PHASE_CURVATURE_DIAGNOSTIC_CONTRACT.md` | `rda_azimuth_phase_curvature_diagnostic.md` |
| `SAR_REFERENCE_BOUNDARY_PARAMETER_MATRIX_CONTRACT.md` | `reference_boundary_parameter_matrix.md` |
| `SAR_REFERENCE_SCENARIO_MATRIX_CONTRACT.md` | `reference_scenario_matrix.md` |
| `SAR_REFERENCE_SNR_MATRIX_CONTRACT.md` | `reference_snr_matrix.md` |
| `SAR_REFERENCE_SNR_SCR_MATRIX_CONTRACT.md` | `reference_snr_scr_matrix.md` |
| `SAR_VARIABLE_PRF_RESAMPLING_CONTRACT.md` | `variable_prf_resampling.md` |
| `SAR_VARIABLE_PRF_RESAMPLING_QUALITY_MATRIX_CONTRACT.md` | `variable_prf_resampling_quality_matrix.md` |
| `docs/sar_public_focused_image_output_contract.md`（错位） | `docs/sar/contracts/public_focused_image_output.md` |

### 6.3 `docs/sar/acceptance/`（51 份验收报告）

> 源均为 `docs/sar_*_acceptance_report.md`，目标 stem 为去 `sar_` 前缀与 `_acceptance_report` 后缀的小写形。

| 目标 stem | 目标 stem | 目标 stem |
|---|---|---|
| `autofocus_phase_truth` | `boundary_parameter_matrix` | `csa_frequency_geometry` |
| `csa_intermediate_truth_executor` | `deterministic_distributed_clutter` | `explicit_calibration_observation` |
| `extended_variable_prf_quality_matrix` | `external_raw_iq_input` | `external_raw_iq_l2_motion_compensation` |
| `external_raw_iq_pulse_trajectory` | `image_quality_completion` | `internal_calibration_executor` |
| `internal_focusing_selector` | `l2_motion_compensation` | `l2_session_integration` |
| `l3_bp` | `l3_bp_session_integration` | `l3_waypoint_trajectory` |
| `missing_pulse_gap_diagnostics` | `missing_pulse_rejection_matrix` | `omega_k_azimuth_inverse_transform` |
| `omega_k_common_support` | `omega_k_grid_reduction` | `omega_k_point_target` |
| `omega_k_reduced_range_axis` | `omega_k_reference_mapping` | `omega_k_reference_phase_compensation` |
| `omega_k_relative_delay_transform` | `omega_k_stolt_geometry` | `omega_k_stolt_interpolation` |
| `omega_k_truth_eligibility` | `omega_k_truth_evaluation_orchestrator` | `omega_k_truth_ingestion` |
| `omega_k_truth_manifest` | `omega_k_truth_payload_digest` | `pga_gradient_truth_comparison` |
| `pga_phase_gradient_estimator` | `pga_support_gradient_truth` | `phase_reference` |
| `radiometric_calibration` | `raw_history_slow_time_resampling` | `rda_aperture_phase_span` |
| `rda_phase_curvature_diagnostics` | `reference_scenario_matrix` | `reference_snr_matrix` |
| `reference_snr_scr_matrix` | `slow_time_resampling` | `slow_time_resampling_executor` |
| `variable_prf_resampling_quality_matrix` | `phase1` | `phase2` |

### 6.4 `docs/sar/decisions/`（32 份决策记录）

**followup_decision（28 = 27 docs + 1 顶层错位）**:

| 目标 stem | 目标 stem | 目标 stem |
|---|---|---|
| `autofocus_phase_truth` | `csa_frequency_geometry` | `csa_intermediate_truth_executor` |
| `distributed_clutter` | `explicit_calibration_observation` | `extended_variable_prf_quality` |
| `internal_calibration_executor` | `internal_focusing_selector` | `joint_snr_scr` |
| `missing_pulse_gap` | `missing_pulse_rejection` | `omega_k_common_support` |
| `omega_k_explicit_phase_compensation` | `omega_k_grid_reduction` | `omega_k_reduced_range_axis` |
| `omega_k_reference_mapping` | `omega_k_relative_delay_transform`（←顶层错位） | `omega_k_stolt_geometry` |
| `omega_k_stolt_interpolation` | `omega_k_truth_ingestion` | `pga_support_gradient_truth` |
| `radiometric_calibration` | `raw_history_resampling` | `rda_diagnostic` |
| `reference_snr` | `slow_time_resampling` | `slow_time_resampling_executor` |
| `variable_prf_quality` | | |

**纯 decision（4）**:

| 源 | 目标 stem |
|---|---|
| `docs/sar_omega_k_atomic_truth_ingestion_gate_decision.md` | `omega_k_atomic_truth_ingestion_gate.md` |
| `docs/sar_omega_k_eligible_truth_evaluation_orchestration_decision.md` | `omega_k_eligible_truth_evaluation_orchestration.md` |
| `docs/sar_omega_k_physical_acceptance_readiness_decision.md` | `omega_k_physical_acceptance_readiness.md` |
| `docs/sar_rda_target_azimuth_offset_decision.md` | `rda_target_azimuth_offset.md` |

### 6.5 `docs/sar/audits/`（10 份审计 / 研究 / 基线 / 收尾）

| 源 | 目标 |
|---|---|
| `docs/sar_azimuth_sampling_audit.md` | `azimuth_sampling.md` |
| `docs/sar_l3_first_order_compensation_audit.md` | `l3_first_order_compensation.md` |
| `docs/sar_phase2_reference_closure_audit.md` | `phase2_reference_closure.md` |
| `docs/sar_remaining_focusing_capability_audit.md` | `remaining_focusing_capability.md` |
| `docs/sar_fft_backend_research.md` | `fft_backend_research.md` |
| `docs/sar_l3_first_order_applicability_matrix_report.md` | `l3_first_order_applicability_matrix.md` |
| `docs/sar_l3_imaging_degradation_baseline_report.md` | `l3_imaging_degradation_baseline.md` |
| `docs/sar_phase1_closeout_report.md` | `phase1_closeout.md` |
| `docs/sar_phase2_reference_imaging_reapproval_report.md` | `phase2_reference_imaging_reapproval.md` |
| `docs/sar_next_incomplete_capability_selection.md` | `next_incomplete_capability_selection.md` |

### 6.6 其它 `docs/` 子目录（9 份）

| 源 | 目标 |
|---|---|
| `docs/public_model_config_manual.md` | `docs/public/model_config_manual.md` |
| `docs/public_module_output_manual.md` | `docs/public/module_output_manual.md` |
| `docs/public_target_input_manual.md` | `docs/public/target_input_manual.md` |
| `docs/review/sar_module_algorithm_capability_review.md` | `docs/review/module_algorithm_capability.md` |
| `docs/review/sar_module_development_process_review.md` | `docs/review/module_development_process.md` |
| `task_plan.md` | `docs/worklog/task_plan.md` |
| `progress.md` | `docs/worklog/progress.md` |
| `findings.md` | `docs/worklog/findings.md` |
| `jsbsim_aircraft_analysis.md` | `docs/reference/jsbsim_aircraft_analysis.md` |

---

## 7. 引用更新清单（共 5 处，移动后统一处理）

| 文件（移动后路径） | 位置 | 旧引用 | 新引用 |
|---|---|---|---|
| `docs/sar/design/module_design.md` | L19 | `` `sar_construction_scheme_complete.md` `` | `` `construction_scheme.md` `` |
| `docs/sar/design/construction_scheme.md` | L185 | `` `SAR_MODULE_DESIGN.md` `` | `` `module_design.md` `` |
| `docs/sar/design/phase1_engineering.md` | L452-453 | `` `sar_construction_scheme_complete.md` `` / `` `SAR_MODULE_DESIGN.md` `` | `` `construction_scheme.md` `` / `` `module_design.md` `` |
| `docs/worklog/task_plan.md`、`findings.md`、`progress.md` | 文档头核心文档清单 | 三份顶层核心文档名 | `../sar/design/*.md` 相对路径 |
| `README.md` | L55-58 | 文档清单（含已失效的 `input_surface_unification_refactor_plan.md`） | 更新为 `docs/` 子树路径，删除失效引用 |

> CLAUDE.md / AGENTS.md 经核查不引用任何具体 SAR 文档路径，无需改动。

---

## 8. 分批执行计划

遵循「每批移动后 `git status` 验证」的稳妥节奏。引用更新集中在批次 6 统一处理。

| 批次 | 内容 | 文件数 | 验证 |
|---|---|---|---|
| 0 | 建目录骨架 + 新增 `docs/README.md`、`docs/sar/README.md` | 8 目录 / 2 新文件 | 目录树核对 |
| 1 | `design/`（核心设计） | 4 | `git status` |
| 2 | `contracts/`（含错位归位） | 44 | `git status` |
| 3 | `acceptance/` | 51 | `git status` |
| 4 | `decisions/`（含顶层错位归位） | 32 | `git status` |
| 5 | `audits/` + `public/` + `review/` + `worklog/` + `reference/` | 19 | `git status` |
| 6 | 引用更新（5 处）+ 顶层清理核查 | 5 | `grep` 复核无残留旧引用 |
| 7 | 全量核对:顶层仅剩 README/CLAUDE/AGENTS;`docs/` 子树完整 | — | `find` 比对 |

**执行约束**:

- 全程使用 `git mv`（已跟踪文件）与 `mv`（未跟踪的 4 份新文件）保留历史。
- 每批结束后 `git status --short` 核对，异常立即中止。
- 不并发修改超过单批范围;批次间串行。

---

## 9. 风险与回退

| 风险 | 影响 | 缓解 |
|---|---|---|
| 重命名幅度大，git rename 识别率下降 | `git log --follow` 仍可追溯单文件，但批量 diff 的 rename 高亮减弱 | 可接受;命名规范化是有意为之，且引用稀疏 |
| 文档被外部 wiki / CI 引用 | 极低概率（仓库内 grep 仅 5 处） | 批次 6 全量 `grep` 复核;若有外部引用，保留旧名软链或更新 |
| planning skill 读写根目录 `task_plan.md` | 移到 `docs/worklog/` 后 skill 可能找不到 | 移动后验证 skill 配置;必要时在 skill 配置中指定新路径 |
| 同名 stem 跨目录（如 `omega_k_stolt_interpolation` 在 acceptance/decisions） | 搜索时需带目录路径 | 预期行为;`docs/sar/README.md` 索引按子能力聚合三件套链接 |

**回退**:每批为独立 commit（或独立 stash 段），任一批出错可 `git checkout` 单批回滚，不影响已完成的批次。

---

## 10. 验收清单

- [ ] 仓库顶层 `.md` 仅剩 `README.md` / `CLAUDE.md` / `AGENTS.md`。
- [ ] `docs/sar/` 下 design(4) / contracts(44) / acceptance(51) / decisions(32) / audits(10) 数量核对一致。
- [ ] `docs/public/`(3) / `docs/review/`(2) / `docs/reference/`(1) / `docs/worklog/`(3) 数量核对一致。
- [ ] 全仓 `grep -rn 'SAR_MODULE_DESIGN\.md\|sar_construction_scheme_complete\.md\|SAR_PHASE1_ENGINEERING_CONTRACT' --include='*.md'` 无残留旧名。
- [ ] `docs/sar/README.md` 索引可点击导航到各子能力三件套。
- [ ] `git log --follow docs/sar/design/module_design.md` 可追溯到原 `SAR_MODULE_DESIGN.md` 历史。

---

## 11. 待评审确认的开放问题

1. **JSBSim 归宿**:`jsbsim_aircraft_analysis.md` 现方案归 `docs/reference/`（新建单文件目录）。备选:直接放 `docs/` 根、或并入 `docs/review/`、或删除（若已过期）。请确认。
2. **纯 report 归类**:4 份纯 report（`applicability_matrix` / `degradation_baseline` / `phase1_closeout` / `phase2_reapproval`）现统一归 `audits/`。其中 `phase2_reference_imaging_reapproval` 偏验收性质，是否改归 `acceptance/`?请确认。
3. **`next_incomplete_capability_selection` 归类**:现归 `audits/`，其性质偏「能力选型决策」，是否改归 `decisions/`?请确认。
4. **public 手册去前缀**:`public_model_config_manual.md` → `model_config_manual.md`（去 `public_`）。备选:保留 `public_` 前缀以强化「公共 API」语义。请确认。
5. **索引文件生成**:是否在本次执行时一并生成 `docs/README.md`（文档区总索引）与 `docs/sar/README.md`（SAR 文档索引，含子能力三件套导航）?建议生成。
6. **本方案文档归宿**:重组完成后，`docs/directory_restructure_plan.md` 保留于 `docs/` 根作为重组历史记录，或移入 `docs/sar/audits/`，或删除。建议保留于 `docs/` 根。

---

*评审通过后，将按第 8 节分批执行。执行期间本文件作为依据，不参与移动。*
