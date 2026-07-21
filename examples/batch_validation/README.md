# 批量场景验证（Batch Validation）

通过参数扫描与跨周期专项序列，验证 AR / EOS / ESR / SAR / SBIRS 五个传感器模块在
真实仿真边界下的物理趋势、身份/资源连续性、配置原子性、拒绝恢复和确定性 replay。
flight_dynamic 不在本框架范围内。

本目录中的可执行程序只使用公开 Session / Adapter / Replay 接口，并按模块使用公开
`config_loader` 或 POD / builder 构造配置。为满足这些外部可观察契约而需要的运行时语义修复
仍由对应生产模块和回归测试拥有，不在 examples 中复制实现。

## 与现有测试的关系

| 维度 | `tests/unit/*_matrix_test.cpp` | 本目录（batch_validation） |
| --- | --- | --- |
| 形态 | GTest 内嵌、`--gtest_filter` 驱动 | 独立可执行程序 |
| 断言 | 硬性 `EXPECT_*`（失败即红） | 契约硬检查 + 物理趋势 warning |
| 输出 | gtest XML / `RecordProperty` | `cycles.csv` + `scenarios.csv` + `checks.csv` + trace |
| 用途 | 回归门控、合同约束 | 泛用性验证、趋势分析、可回放回归 |
| 回放 | 部分（replay test） | 每场景录制 + 确定性回放 |

两者互补：matrix_test 是 CI 门控，本框架是"用大量场景证明模块泛用性"的数据采集与分析工具。

当前共有 230 个场景：保留 199 个 sweep（AR 52、EOS 36、ESR 48、SAR 36、SBIRS 27），
并新增 31 个 sequence（AR 6、EOS 6、ESR 6、SAR 6、SBIRS 7）。AR 验证器在
构造每个会话前显式设置 `enable_physics_detection=true`、`enable_physical_rcs=true`
和 `physics_mix_ratio=1.0`，不依赖 JSON 默认值。

## 目录结构

```
batch_validation/
├── README.md                  本文件
├── batch_csv_writer.h         共享：流式 CSV writer（仿 flight_dynamic/orbit_quality_csv.cpp）
├── batch_replay.h             共享：ReplayTraceWriter 工厂 + 回放结果摘要
├── batch_assertions.h         共享：WarningCollector + 统计辅助（Mean/Percentile/单调性）
├── batch_cli.h                共享：显式 suite/scenario/output CLI
├── batch_checks.h             共享：结构化契约检查与 checks.csv
├── ar_batch_validation.cpp    AR sweep + 6 个跨周期专项场景
├── eos_batch_validation.cpp   EOS sweep + 6 个跨周期专项场景
├── esr_batch_validation.cpp   ESR sweep + 6 个跨周期专项场景
├── sar_batch_validation.cpp   SAR sweep + 6 个跨周期专项场景
├── sbirs_batch_validation.cpp SBIRS sweep + 7 个跨周期专项场景
└── analyze_batch_results.py   分析脚本：读 scenarios.csv → 趋势表 + 高亮告警 + 趋势图
```

## 构建与运行

```bash
# 构建（需先完成项目标准 bootstrap + configure 步骤）
bash scripts/bootstrap_conan.sh llvm-ninja-release-local
cmake --preset llvm-ninja-release-local
cmake --build --preset llvm-ninja-release-local --target \
    ar_batch_validation eos_batch_validation esr_batch_validation sar_batch_validation \
    sbirs_batch_validation

# 默认运行 all（sweep + sequence）
./build/llvm-ninja-release-local/bin/ar_batch_validation
./build/llvm-ninja-release-local/bin/eos_batch_validation
./build/llvm-ninja-release-local/bin/esr_batch_validation
./build/llvm-ninja-release-local/bin/sar_batch_validation
./build/llvm-ninja-release-local/bin/sbirs_batch_validation

# 只运行 sequence、单个精确 ID，或列出场景
./build/llvm-ninja-release-local/bin/ar_batch_validation --suite sequence
./build/llvm-ninja-release-local/bin/ar_batch_validation \
    --scenario ar_seq_invalid_input_recovery --output-dir /tmp/1q/ar-invalid
./build/llvm-ninja-release-local/bin/ar_batch_validation --list-scenarios

# CI 注册的五个专项套件（不重复执行 sweep）
ctest --preset llvm-ninja-release-local -L batch_validation --output-on-failure -j 4

# 分析
python3 examples/batch_validation/analyze_batch_results.py
```

五个程序使用同一组显式参数：

- `--suite sweep|sequence|all`：默认 `all`。
- `--scenario <exact-id>`：只运行精确匹配的一个场景。
- `--list-scenarios`：列出所选 suite 中的场景 ID 后退出。
- `--output-dir <path>`：覆盖默认输出目录。

旧位置参数入口已删除；位置参数和未知参数都会返回非零。

## 专项场景目录

| 模块 | sequence 场景 ID |
| --- | --- |
| AR | `ar_seq_two_target_crossing`、`ar_seq_crossing_with_pulsed_jammer`、`ar_seq_tws_stt_tws`、`ar_seq_power_cycle`、`ar_seq_invalid_input_recovery`、`ar_seq_invalid_patch_atomic` |
| EOS | `eos_seq_two_target_focal_crossing`、`eos_seq_day_twilight_night`、`eos_seq_fused_ir_visible_fused`、`eos_seq_scan_rate_retask`、`eos_seq_power_cycle`、`eos_seq_invalid_input_recovery` |
| ESR | `esr_seq_two_emitter_angular_crossing`、`esr_seq_dense_emitters_with_silence`、`esr_seq_mode_switch`、`esr_seq_scan_bounds_retask`、`esr_seq_power_cycle`、`esr_seq_invalid_input_recovery` |
| SAR | `sar_seq_multi_scatterer_resolution`、`sar_seq_squint_gate_recovery`、`sar_seq_raw_to_image`、`sar_seq_invalid_runtime_atomic`、`sar_seq_invalid_input_recovery`、`sar_seq_low_snr_recovery` |
| SBIRS | `sbirs_seq_two_target_crossing_two_locks`、`sbirs_seq_three_target_one_lock_handoff`、`sbirs_seq_boost_maneuver_nis_reacquire`、`sbirs_seq_cue_latency_cross_velocity`、`sbirs_seq_occultation_reappearance`、`sbirs_seq_standby_mission_retask`、`sbirs_seq_invalid_input_recovery` |

## 输出布局

```
/tmp/1q/batch_validation/
├── airborne_radar/
│   ├── cycles.csv             周期级指标（每周期一行）
│   ├── scenarios.csv          场景汇总（每场景一行，含参数/聚合指标/replay_ok/warnings）
│   ├── checks.csv             专项契约检查（期望值、实际值、通过状态、严重度）
│   └── traces/<scenario_id>/  每场景一个可回放 trace 目录（manifest.json + events/）
├── electro_optical_sensor/    同上
├── electronic_surveillance_radar/  同上
├── sar/                       同上（单周期聚焦，cycles.csv 每场景一行）
└── sbirs_sensor/              同上（每场景两周期，覆盖捕获与跟踪交接）
```

## 关键技术点

### 用 TraceSession 录制可回放 trace

批量程序直接用 `XxxTraceSession(config, options)`（而非 `<Module>Module` 包装类，
后者内部用普通 Session 无法录制可回放 trace）。`options.replay_writer` 指向由
`batch_validation::MakeReplayWriter` 创建的 `ReplayTraceWriter`：

- `TraceSession` 内部在构造时自动写 `session_config`，每次 `StepWithResult` 自动写
  `cycle_input` + `cycle_output`，`ApplyRuntimeConfig` 时写 `runtime_config_patch`，
  校验失败时写 `failure_marker`。无需手动 `WriteEvent`。
- 回放前必须 `replay_writer->Flush()`，且建议 writer 先析构（文件句柄释放）。
- 每个场景用独立 `trace_dir` + 唯一 manifest；`manifest.module` 必须精确等于模块
  字符串常量（`batch_replay.h::ModuleName`），否则回放兼容性检查失败。

回放由各模块的 `ReplayXxxTrace(trace_dir)` 完成：它重建 Session、重放每个 cycle_input、
重新执行 `StepWithResult`，并与 trace 中记录的 `cycle_output` 逐字段比较
（`divergence_found`）。failure marker 会进入报告但不会截断回放；marker 后的恢复周期仍须
逐个比较。任何分叉都会令 `replay_ok=0` 并使程序和结果分析器返回非零。

### 硬契约与物理 warning

以下属于硬失败：replay 分叉或比较数量不完整、结构化 validation/abort reason 不符、
failure marker 数不符、身份/通道/产品连续性错误、配置部分污染、非执行周期产生 lifecycle
事件或推进 recorder 状态。预期无效输入被正确拒绝不记 error；只有“未按预期拒绝”才记 error。

对关键物理趋势做检查，违反时记录 `kWarning`（不中断程序）：

| 模块 | 软断言 |
| --- | --- |
| AR | 近距离 + 高 RCS 应有确认轨迹；`match_rate` ∈ [0,1]；距离↑ 时确认数单调↓ |
| EOS | 高对比度检出率 ≥ 低对比度；夜间可见光 SNR 显著低于红外 |
| ESR | 假设置信度 / 真值匹配率 ∈ [0,1]；占用率↑ 时受扰观测↑ |
| SAR | 聚焦阶段达 `kL1RdaImage`；图像质量指标有效；带宽↑ → 距离分辨率↓ |
| SBIRS | 两周期均执行；温度↑ 时红外 SNR 不下降；覆盖可检出与门限下不可检出场景 |

Release 模式下五模块 `--suite all` 的总运行时间目标不超过 120 秒。不得通过放宽 replay
比较、阈值、skip 或 unstable 标记消除失败。

## CSV Schema（场景汇总 scenarios.csv 通用列）

| 列 | 含义 |
| --- | --- |
| `scenario_id` | 场景标识（含参数编码） |
| `suite` / `scenario_family` | `sweep` 或 `sequence` 及专项族 |
| `executed_cycles` | 实际执行的周期数 |
| `replay_ok` | 回放是否成功（0=分叉/失败） |
| `replay_compared` | 回放比对过的 cycle_output 数 |
| `replay_divergence` | 是否检测到输出分叉 |
| `warning_count` / `error_count` | 软断言告警计数 |
| `expected_failure_count` | 本场景预期的拒绝/abort 数 |
| `contract_check_count` / `contract_failure_count` | 结构化契约检查总数/硬失败数 |
| `failure_marker_count` | trace 中实际 failure marker 数 |
| `warnings` | 拼接的告警文本（`[W]`/`[E]` 前缀） |

模块特有指标列见各程序文件头部的 `kScenarioHeader` 常量。`cycles.csv` 还包含 `suite`、
`scenario_family`、`phase`；`checks.csv` 固定记录 `scenario_id,phase,cycle_index,check_id,`
`expected,actual,passed,severity`。

## 手动回放复盘

每个场景的 trace 可事后手动回放（用于调试或回归）。trace 目录结构遵循
`include/1q/replay/ReplayTrace.h` 约定（`manifest.json` + `events/*.events.jsonl`）。

```cpp
// 在你自己的程序中回放某个 AR 场景：
#include "1q/airborne_radar/session/ArReplaySession.h"
auto result = airborne_radar::session::ReplayArTrace(
    "/tmp/1q/batch_validation/airborne_radar/traces/ar_r008km_rcs5.00_n3");
// result.ok / result.playback.divergence_found / result.first_error
```

EOS / ESR / SAR / SBIRS 同理（`ReplayEosTrace` / `ReplayEsrTrace` / `ReplaySarTrace` /
`ReplaySbirsTrace`）。

## 当前物理边界

- **ESR 距离/占用率不敏感**：50MW 辐射源在 10–100km 内均远超 ESR 截获门限，故
  `truth_match_rate` 在所有距离下一致。这反映模块在强信号下的稳定行为，而非缺陷。
  要观察距离衰减需进一步降低辐射功率（超出本框架的"配置面"范畴）。
