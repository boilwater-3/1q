# 计划：消除 PipelineConfig 判别联合，统一为物理参数配置

## Context

`PipelineConfig` 当前同时持有 `semantic::*Config` 和 `expert::ExpertPipelineConfig`
两棵完整配置树，用 `PipelineConfigModel model` 枚举选择哪棵有效。这是一个不安全的
判别联合（discriminated union）：两份数据始终分配在内存中，但只有一份有效，没有任何
编译期保护防止读到废弃的那份。

对于一个仿真模型库，物理参数（peak_power_w、noise_figure_db 等）是主要接口。语义层
（Profile enum）应作为 Builder 的便捷输入，在 `Build()` 时翻译成物理值，不进入
wire format（RadarSessionConfig）。

**目标**：`ExpertPipelineConfig`（物理参数）成为唯一的配置表示；语义 Profile enum
保留在公开 API 作为 Builder 输入类型；semantic 配置结构体迁入 src/ 作为 Builder
私有状态。

---

## 关键文件

| 文件 | 变更性质 |
|------|---------|
| `include/1q/airborne_radar/config/PipelineConfig.h` | 删除 model/semantic 字段，保留 expert + orientation |
| `include/1q/airborne_radar/config/ConfigModel.h` | **删除** |
| `include/1q/airborne_radar/config/expert/tracking/TrackingConfig.h` | 新增 speed_decay / rcs_decay 字段 |
| `include/1q/airborne_radar/config/RadarSessionConfigBuilder.h` | Build() 翻译逻辑，semantic 结构体私有化 |
| `include/1q/airborne_radar/config/RadarExpertSessionConfigBuilder.h` | 小调整（字段路径） |
| `include/1q/airborne_radar/config/airborne_radar_config.hpp` | 移除 ConfigModel、semantic config 结构头文件 |
| `src/airborne_radar/session/RadarSessionConfigBuilder.cpp` | Build() 实现翻译逻辑 |
| `src/airborne_radar/session/RuntimeConfigResolver.cpp` | beam_control.radar_orientation → orientation |
| `src/airborne_radar/session/RadarSessionConfigPresets.cpp` | 移除 model 字段赋值 |
| `src/airborne_radar/signal/pipeline/config/InternalPipelineConfig.h` | 删除 kSemantic 分支 |
| `tests/unit/ar_session_config_builder_test.cpp` | 更新断言 |
| `tests/contract/ar_public_api_convenience_test.cpp` | 更新断言 |
| `examples/ar/ar_quick_start.cpp` | 如涉及 model 字段则更新 |

Semantic 结构体（DetectionConfig、TrackingConfig、LifecycleConfig、BeamControlConfig）
迁移到 `src/airborne_radar/session/` 作为 Builder 私有头文件。
Profile enum 头文件（DetectionProfiles.h、TrackingProfiles.h、LifecycleProfiles.h）
**保留**在 `include/`，是 Builder 公开输入类型。

---

## 实施步骤

### Step 1：扩展 ExpertPipelineConfig 覆盖所有物理参数

**`expert/tracking/TrackingConfig.h`** 新增两个字段，使内部 profile tuning 变为显式控制：
```cpp
struct TrackingConfig {
  bool enable_kalman_filter{true};
  float kalman_measurement_noise_std{10.0f};
  KalmanUpdateBackend kalman_update_backend{KalmanUpdateBackend::kStandardKfJoseph};
  float speed_decay_ratio_on_loss{1.0f};   // 新增：丢失周期速度衰减系数
  float rcs_decay_ratio_on_loss{1.0f};     // 新增：丢失周期 RCS 衰减系数
};
```

### Step 2：重构 PipelineConfig

```cpp
// 新 PipelineConfig（唯一表示）
struct PipelineConfig {
  expert::ExpertPipelineConfig expert{};
  model::RadarOrientationConfig orientation{};  // 替代 semantic::BeamControlConfig
};
```

删除：`model`、`detection`（semantic）、`beam_control`（semantic）、
`tracking`（semantic）、`lifecycle`（semantic）字段。
删除 `ConfigModel.h`。

### Step 3：将 semantic→expert 翻译移入 Build()

`RadarSessionConfigBuilder` 保留 semantic 字段作为 Builder 内部状态（不放入
`PipelineConfig`），`Build()` 调用已在 `InternalPipelineConfig.h` 中存在的：
- `ResolveDetectionEngineering(semantic::DetectionConfig)` → 填写 `expert.detection`
- `ResolveTrackingEngineering(semantic::TrackingConfig)` → 填写 `expert.tracking`（含新字段）
- `ResolveLifecycleEngineering(semantic::LifecycleConfig)` → 填写 `expert.lifecycle`
- 天线方向图、RCS 融合映射同步迁入

这些函数签名不变，仅调用方从 SignalPipeline 内部移到 Builder。

### Step 4：迁移 semantic 结构体至 src/

将以下文件从 `include/1q/airborne_radar/config/semantic/` 移至
`src/airborne_radar/session/config/`（Builder 私有）：
- `DetectionConfig.h`
- `TrackingConfig.h`
- `LifecycleConfig.h`
- `BeamControlConfig.h`
- `AntennaPatternConfig.h`（semantic 版本）

保留在 `include/`：
- `profiles/DetectionProfiles.h`（RadarHardwareProfile、DetectionIntentProfile 等 enum）
- `profiles/TrackingProfiles.h`
- `profiles/LifecycleProfiles.h`

### Step 5：简化 InternalPipelineConfig.h

- `BuildBaselineInternalPipelineConfig()` 删除 `kSemantic` 分支，只保留
  `expert.detection` 路径（现在 `Build()` 保证输入总是物理参数）
- `ResolveInternalProfileFromPublicConfig()` **删除**（Profile tuning 已由 Builder
  写入 expert 字段，内部无需再读 semantic 字段路由）
- `ApplyInternalProfileTuning()` **删除**（tuning 值已在 Build() 时写入
  `expert.tracking.speed_decay_ratio_on_loss` 等字段）

### Step 6：更新 RuntimeConfigResolver

```cpp
// 旧
pipeline_config->beam_control.radar_orientation.work_sub_mode = patch.work_sub_mode;

// 新
pipeline_config->orientation.work_sub_mode = patch.work_sub_mode;
```

所有 `beam_control.radar_orientation.*` → `orientation.*`。

### Step 7：更新 ExpertSessionConfigBuilder、Presets、测试、示例

- `RadarExpertSessionConfigBuilder`：字段路径更新（`expert.beam_control` →
  `expert.beam_control`，orientation → `orientation`）
- `RadarSessionConfigPresets.cpp`：删除 `config.pipeline_config.model = kSemantic` 赋值
- 测试/示例：更新字段路径，移除对 `model` 字段的引用

---

## 验证

```bash
preset=llvm-ninja-debug-local
cmake --preset "$preset" >/tmp/1q-cmake.log 2>&1 || { tail -n 80 /tmp/1q-cmake.log; false; }
cmake --build --preset "$preset" >/tmp/1q-build.log 2>&1 || { tail -n 80 /tmp/1q-build.log; false; }
ctest --preset "$preset" --output-on-failure
```

重点确认：
- `ar_session_config_builder_test` 全通
- `ar_public_api_convenience_test` 全通
- `ar_signal_detection_test` 全通
- `public_headers_smoke_test` 全通（确认公开头文件无 semantic struct 残留）
