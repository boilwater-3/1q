# SAR 模块文档索引

本目录收纳 SAR 模块的全部文档，按**生命周期**分 5 个子目录。命名约定：去除 `SAR_` 前缀与类型后缀，全小写；**目录表达文档类型，文件名表达子能力**。

## 子目录

| 目录 | 文档类型 | 数量 |
|---|---|---|
| [`design/`](design/) | 核心设计：接口形态 / 算法口径 / 阶段工程契约 | 4 |
| [`contracts/`](contracts/) | 子能力合约：边界定义与冻结口径 | 49 |
| [`acceptance/`](acceptance/) | 验收报告：交付确认 | 51 |
| [`decisions/`](decisions/) | 决策记录：后续方向 / 选型门 | 32 |
| [`audits/`](audits/) | 审计 / 研究 / 基线 / 收尾 | 21 |

## 核心设计（权威入口）

- [`design/module_design.md`](design/module_design.md) — SAR 模块接口与实现形态（v2.8）
- [`design/construction_scheme.md`](design/construction_scheme.md) — 算法层数学公式与审批口径
- [`design/phase1_engineering.md`](design/phase1_engineering.md) — Phase 1 工程契约
- [`design/phase2_reference_imaging.md`](design/phase2_reference_imaging.md) — Phase 2 参考级成像契约

## 子能力三件套速查

同一子能力通常有成对的**合约**（边界）/ **验收**（确认）/ **决策**（方向）文档，文件名共享 stem。下表 ✓ 表示该 stem 在对应目录存在文档。

| 子能力 stem | 合约 | 验收 | 决策 |
|---|:--:|:--:|:--:|
| `autofocus_phase_error_reference` | ✓ |   |   |
| `autofocus_phase_truth` |   | ✓ | ✓ |
| `boundary_parameter_matrix` |   | ✓ |   |
| `csa_frequency_geometry` |   | ✓ | ✓ |
| `csa_intermediate_truth_executor` |   | ✓ | ✓ |
| `csa_math_reference` | ✓ |   |   |
| `csa_phase_function_intermediate_truth` | ✓ |   |   |
| `deterministic_distributed_clutter` | ✓ | ✓ |   |
| `distributed_clutter` |   |   | ✓ |
| `explicit_calibration_observation` | ✓ | ✓ | ✓ |
| `extended_variable_prf_quality` |   |   | ✓ |
| `extended_variable_prf_quality_matrix` |   | ✓ |   |
| `extended_variable_prf_resampling_quality_matrix` | ✓ |   |   |
| `external_raw_iq_input` | ✓ | ✓ |   |
| `external_raw_iq_l2_motion_compensation` | ✓ | ✓ |   |
| `external_raw_iq_pulse_trajectory` | ✓ | ✓ |   |
| `image_quality_completion` | ✓ | ✓ |   |
| `internal_calibration_executor` |   | ✓ | ✓ |
| `internal_focusing_selector` | ✓ | ✓ | ✓ |
| `internal_session_calibration_request` | ✓ |   |   |
| `internal_slow_time_resampling_request` | ✓ |   |   |
| `joint_snr_scr` |   |   | ✓ |
| `l2_motion_compensation` | ✓ | ✓ |   |
| `l2_session_integration` | ✓ | ✓ |   |
| `l3_bp` | ✓ | ✓ |   |
| `l3_bp_session_integration` | ✓ | ✓ |   |
| `l3_waypoint_trajectory` | ✓ | ✓ |   |
| `missing_pulse_gap` |   |   | ✓ |
| `missing_pulse_gap_diagnostics` | ✓ | ✓ |   |
| `missing_pulse_rejection` |   |   | ✓ |
| `missing_pulse_rejection_matrix` | ✓ | ✓ |   |
| `omega_k_atomic_truth_ingestion_gate` |   |   | ✓ |
| `omega_k_azimuth_inverse_transform` |   | ✓ |   |
| `omega_k_common_stolt_support` | ✓ |   |   |
| `omega_k_common_support` |   | ✓ | ✓ |
| `omega_k_complex_stolt_interpolation` | ✓ |   |   |
| `omega_k_eligible_truth_evaluation_orchestration` |   |   | ✓ |
| `omega_k_explicit_grid_reduction` | ✓ |   |   |
| `omega_k_explicit_phase_compensation` |   |   | ✓ |
| `omega_k_grid_reduction` |   | ✓ | ✓ |
| `omega_k_independent_physical_truth_ingestion` | ✓ |   |   |
| `omega_k_math_reference` | ✓ |   |   |
| `omega_k_physical_acceptance_readiness` |   |   | ✓ |
| `omega_k_point_target` |   | ✓ |   |
| `omega_k_point_target_image_acceptance` | ✓ |   |   |
| `omega_k_reduced_range_axis` | ✓ | ✓ | ✓ |
| `omega_k_reference_mapping` |   | ✓ | ✓ |
| `omega_k_reference_phase_absolute_range` | ✓ |   |   |
| `omega_k_reference_phase_compensation` |   | ✓ |   |
| `omega_k_relative_delay_transform` | ✓ | ✓ | ✓ |
| `omega_k_stolt_geometry` |   | ✓ | ✓ |
| `omega_k_stolt_interpolation` |   | ✓ | ✓ |
| `omega_k_truth_eligibility` |   | ✓ |   |
| `omega_k_truth_evaluation_orchestrator` |   | ✓ |   |
| `omega_k_truth_ingestion` |   | ✓ | ✓ |
| `omega_k_truth_manifest` |   | ✓ |   |
| `omega_k_truth_payload_digest` |   | ✓ |   |
| `omega_k_truth_payload_digest_verification` | ✓ |   |   |
| `pga_gradient_estimator_truth_comparison` | ✓ |   |   |
| `pga_gradient_truth_comparison` |   | ✓ |   |
| `pga_phase_gradient_estimator` |   | ✓ |   |
| `pga_support_gradient_truth` | ✓ | ✓ | ✓ |
| `phase_reference` | ✓ | ✓ |   |
| `phase1` |   | ✓ |   |
| `phase2` |   | ✓ |   |
| `public_focused_image_output` | ✓ |   |   |
| `radiometric_calibration` | ✓ | ✓ | ✓ |
| `raw_history_resampling` |   |   | ✓ |
| `raw_history_slow_time_resampling` | ✓ | ✓ |   |
| `rda_aperture_phase_span` |   | ✓ |   |
| `rda_aperture_phase_span_diagnostic` | ✓ |   |   |
| `rda_azimuth_phase_curvature_diagnostic` | ✓ |   |   |
| `rda_diagnostic` |   |   | ✓ |
| `rda_phase_curvature_diagnostics` |   | ✓ |   |
| `rda_target_azimuth_offset` |   |   | ✓ |
| `reference_boundary_parameter_matrix` | ✓ |   |   |
| `reference_scenario_matrix` | ✓ | ✓ |   |
| `reference_snr` |   |   | ✓ |
| `reference_snr_matrix` | ✓ | ✓ |   |
| `reference_snr_scr_matrix` | ✓ | ✓ |   |
| `slow_time_resampling` |   | ✓ | ✓ |
| `slow_time_resampling_executor` |   | ✓ | ✓ |
| `variable_prf_quality` |   |   | ✓ |
| `variable_prf_resampling` | ✓ |   |   |
| `variable_prf_resampling_quality_matrix` | ✓ | ✓ |   |

> - 表中 stem 即各目录下文件名（不含 `.md`）。例如 `omega_k_stolt_interpolation` 对应 `acceptance/omega_k_stolt_interpolation.md`、`decisions/omega_k_stolt_interpolation.md`，合约层为 `contracts/omega_k_complex_stolt_interpolation.md`。
> - 合约层文件名可能含额外修饰词（如 `omega_k_complex_stolt_interpolation`、`omega_k_point_target_image_acceptance`），按字母序在表中定位。
> - 审计 / 研究 / 基线 / 收尾 / 选型类文档归 [`audits/`](audits/)，未列入本表。
