# 批量场景验证（Batch Validation）

通过多场景参数扫描，验证 AR / EOS / ESR / SAR / SBIRS 五个传感器模块在不同物理条件下的
泛用性与正确性。每个模块一个独立可执行程序：构造多组场景 → 跑完仿真周期 →
采集周期级与场景汇总级 CSV 指标 → 对关键物理趋势做软断言 → 每个场景用 `XxxTraceSession`
录制可回放 trace，并立即用 `ReplayXxxTrace` 做确定性回归（分叉检测）。

本目录是 **examples 层**：只使用对外公开的 Session / Adapter / Replay 接口，
并按模块使用公开 `config_loader` 或 POD / builder 构造配置，不修改任何 `include/` 或 `src/`。

## 与现有测试的关系

| 维度 | `tests/unit/*_matrix_test.cpp` | 本目录（batch_validation） |
| --- | --- | --- |
| 形态 | GTest 内嵌、`--gtest_filter` 驱动 | 独立可执行程序 |
| 断言 | 硬性 `EXPECT_*`（失败即红） | 软断言（违反记 kWarning 不中断） |
| 输出 | gtest XML / `RecordProperty` | CSV（周期级 + 场景级）+ trace |
| 用途 | 回归门控、合同约束 | 泛用性验证、趋势分析、可回放回归 |
| 回放 | 部分（replay test） | 每场景录制 + 确定性回放 |

两者互补：matrix_test 是 CI 门控，本框架是"用大量场景证明模块泛用性"的数据采集与分析工具。

当前矩阵共 199 个场景：AR 52、EOS 36、ESR 48、SAR 36、SBIRS 27。AR 验证器在
构造每个会话前显式设置 `enable_physics_detection=true`、`enable_physical_rcs=true`
和 `physics_mix_ratio=1.0`，不依赖 JSON 默认值。

## 目录结构

```
batch_validation/
├── README.md                  本文件
├── batch_csv_writer.h         共享：流式 CSV writer（仿 flight_dynamic/orbit_quality_csv.cpp）
├── batch_replay.h             共享：ReplayTraceWriter 工厂 + 回放结果摘要
├── batch_assertions.h         共享：WarningCollector + 统计辅助（Mean/Percentile/单调性）
├── ar_batch_validation.cpp    AR 场景扫描（距离 × RCS × 目标数 × 探测阈值）
├── eos_batch_validation.cpp   EOS 场景扫描（距离 × 红外对比度 × 光照）
├── esr_batch_validation.cpp   ESR 场景扫描（辐射源距离 × 载频 × 频谱占用率）
├── sar_batch_validation.cpp   SAR 场景扫描（带宽 × 斜距 × 方位脉冲数）
├── sbirs_batch_validation.cpp SBIRS 场景扫描（距离 × 温度 × 投影面积）
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

# 运行（默认输出到 /tmp/1q/batch_validation/<module>/）
./build/llvm-ninja-release-local/bin/ar_batch_validation
./build/llvm-ninja-release-local/bin/eos_batch_validation
./build/llvm-ninja-release-local/bin/esr_batch_validation
./build/llvm-ninja-release-local/bin/sar_batch_validation
./build/llvm-ninja-release-local/bin/sbirs_batch_validation

# 分析
python3 examples/batch_validation/analyze_batch_results.py
```

每个程序接受可选的 `[output_dir]` 参数覆盖默认输出目录。

## 输出布局

```
/tmp/1q/batch_validation/
├── airborne_radar/
│   ├── cycles.csv             周期级指标（每周期一行）
│   ├── scenarios.csv          场景汇总（每场景一行，含参数/聚合指标/replay_ok/warnings）
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
（`divergence_found`）。任何分叉记入场景汇总 CSV 的 `replay_ok` 列。

### 软断言策略

对关键物理趋势做检查，违反时记录 `kWarning`（不中断程序）：

| 模块 | 软断言 |
| --- | --- |
| AR | 近距离 + 高 RCS 应有确认轨迹；`match_rate` ∈ [0,1]；距离↑ 时确认数单调↓ |
| EOS | 高对比度检出率 ≥ 低对比度；夜间可见光 SNR 显著低于红外 |
| ESR | 假设置信度 / 真值匹配率 ∈ [0,1]；占用率↑ 时受扰观测↑ |
| SAR | 聚焦阶段达 `kL1RdaImage`；图像质量指标有效；带宽↑ → 距离分辨率↓ |
| SBIRS | 两周期均执行；温度↑ 时红外 SNR 不下降；覆盖可检出与门限下不可检出场景 |

回放分叉也记为 warning（非 error）：这是模块在边界场景下的确定性属性，应被记录与
高亮，但不阻塞批量运行。只有 `kError`（配置加载失败、Adapter::Build 失败、无周期执行）
才使程序返回非零退出码。

## CSV Schema（场景汇总 scenarios.csv 通用列）

| 列 | 含义 |
| --- | --- |
| `scenario_id` | 场景标识（含参数编码） |
| `executed_cycles` | 实际执行的周期数 |
| `replay_ok` | 回放是否成功（0=分叉/失败） |
| `replay_compared` | 回放比对过的 cycle_output 数 |
| `replay_divergence` | 是否检测到输出分叉 |
| `warning_count` / `error_count` | 软断言告警计数 |
| `warnings` | 拼接的告警文本（`[W]`/`[E]` 前缀） |

模块特有指标列见各程序文件头部的 `kScenarioHeader` 常量。周期级 `cycles.csv` 的列定义见
`kCycleHeader`。

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

## 当前已知发现

- **ESR 近距离（10km）场景回放分叉**：高功率辐射源在近距离时，重放逐字段严格比较
  会因观测时间戳等字段产生确定性漂移。已记为 warning 不阻塞。这是模块确定性属性，
  非框架问题。
- **ESR 距离/占用率不敏感**：50MW 辐射源在 10–100km 内均远超 ESR 截获门限，故
  `truth_match_rate` 在所有距离下一致。这反映模块在强信号下的稳定行为，而非缺陷。
  要观察距离衰减需进一步降低辐射功率（超出本框架的"配置面"范畴）。
