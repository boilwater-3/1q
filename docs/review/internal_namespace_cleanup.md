# `internal` 命名空间清理方案

Status: draft

> 初始审查日期: 2026-07-02
> 复核日期: 2026-07-06
> 审查范围: `src/` 下非 generated 代码的 `namespace internal` / `oneq::internal::*` 使用情况

---

## 一、全景扫描

当前 checkout 中，`src/` 非 generated 代码共有 **14 个头文件**和 **4 个 .cpp 文件**直接声明 `internal` 命名空间。它们实际上扮演三个不同角色，每个角色的问题和处理方式不同。

### 角色一：短路径 Alias（Dual-Alias 形态）

| 文件 | 主要命名空间 | `internal` Alias |
|---|---|---|
| `src/common/numerics/Constants.h` | `oneq::common::numerics` | `oneq::internal::numerics`（逐行 `using`） |
| `src/common/numerics/ClampUtils.h` | 同上 | 同上 |
| `src/common/numerics/NumericGuard.h` | 同上 | 同上 |
| `src/common/geometry/GeometryTransform.h` | `oneq::common::geometry` | `oneq::internal::geometry`（逐行 `using`） |
| `src/common/coordinate/CoordinateUtils.h` | `oneq::common::coordinate_utils` | `oneq::internal::coordinate_utils`（逐行 `using`） |

**证据 - Constants.h 结构：**

```cpp
namespace oneq {
namespace common {
namespace numerics {
constexpr long double kPi = ...;
template <typename T> inline T DegToRad(T angle_deg) { ... }
}  // namespace numerics
}  // namespace common
}  // namespace oneq

namespace oneq {
namespace internal {
namespace numerics {
using ::oneq::common::numerics::kPi;
using ::oneq::common::numerics::DegToRad;
}  // namespace numerics
}  // namespace internal
}  // namespace oneq
```

**关键发现：** 排除本审查文档后，当前代码没有任何 `.cpp` 或 `.h` 引用 `oneq::internal::numerics::`、`oneq::internal::geometry::` 或 `oneq::internal::coordinate_utils::` 路径。这些 alias 是死代码。

### 角色二：伪装的跨模块公共设施

这些文件只有 `oneq::internal::*` 或 `oneq::trace::internal::*`，没有 `oneq::common::*` 对应，但功能上已经是 `src/common/` 跨模块共享设施。

| 文件 | 当前命名空间 | 当前引用范围 |
|---|---|---|
| `src/common/validation/ValidationUtils.h` | `oneq::internal::validation` | AR / ESR / EOS / SAR 校验、runtime config resolver、config mapper、coordinate transform、external adapter |
| `src/common/timing/TimingRegimeModel.h` | `oneq::internal::timing` | AR detection timing、ESR intercept timing、相关单测 |
| `src/common/rcs/RcsPhysics.h` | `oneq::internal::rcs` | AR propagation / detection、RCS 单测 |
| `src/common/atmosphere/StandardAtmosphere.h` | `oneq::internal::atmosphere` | common atmosphere 单测 |
| `src/common/atmosphere/AtmospherePhysics.h` | `oneq::internal::atmosphere` | AR detection atmospheric loss、atmosphere 单测 |
| `src/common/trace/TimeUtils.h` | `oneq::internal::trace` | `TraceSink`、`ReplayTrace` |
| `src/common/trace/JsonFormatUtils.h` | `oneq::trace::internal` | `ReplayTrace` |
| `src/common/output/OutputFrameUtils.h` | `oneq::internal::output` | ESR output manager、output frame 单测 |
| `src/common/runtime/RuntimeCycleExecutor.h` | `oneq::internal::runtime` | AR / ESR controller |

当前非 generated `src/` 调用方清单如下，后续迁移必须按 live `rg` 重新确认，不能只改示例调用方：

```text
src/airborne_radar/config/mapping/RuntimePatchMapper.cpp
src/airborne_radar/environment/PropagationModel.cpp
src/airborne_radar/runtime/ArController.cpp
src/airborne_radar/session/ArExternalInputAdapter.cpp
src/airborne_radar/session/ArExternalOutputAdapter.cpp
src/airborne_radar/session/ArInputValidation.cpp
src/airborne_radar/signal/pipeline/ControlProfileEffects.cpp
src/airborne_radar/signal/pipeline/DetectionExecution.cpp
src/common/coordinate/AttitudeTransform.cpp
src/common/coordinate/PositionTransform.cpp
src/common/coordinate/VelocityTransform.cpp
src/common/replay/ReplayTrace.cpp
src/common/trace/TraceSink.cpp
src/electro_optical_sensor/runtime/EosRuntimeConfigResolver.cpp
src/electro_optical_sensor/session/EosExternalOutputAdapter.cpp
src/electro_optical_sensor/session/EosInputValidation.cpp
src/electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.cpp
src/electronic_surveillance_radar/pipeline/InterceptDetectionExecutor.h
src/electronic_surveillance_radar/runtime/EsrController.cpp
src/electronic_surveillance_radar/runtime/EsrOutputManager.cpp
src/electronic_surveillance_radar/session/EsrExternalInputAdapter.cpp
src/electronic_surveillance_radar/session/EsrExternalOutputAdapter.cpp
src/electronic_surveillance_radar/session/EsrInputValidation.cpp
src/electronic_surveillance_radar/session/EsrResolutionRules.cpp
src/electronic_surveillance_radar/session/EsrRuntimeConfigResolver.cpp
src/sar/session/SarRuntimeConfigResolver.cpp
```

### 角色三：.cpp 文件中的实现代码

```cpp
// RcsPhysics.cpp
namespace oneq {
namespace internal {
namespace rcs {
float ComputeCylinderRcs(...) { ... }
}  // namespace rcs
}  // namespace internal
}  // namespace oneq
```

有些文件还包含匿名 namespace helper。迁移时应区分两件事：

- 将跨文件符号所属命名空间从 `oneq::internal::*` 改成 `oneq::common::*`。
- 保留 `.cpp` 内真正只服务本翻译单元的匿名 namespace，不把 helper 意外导出为具名命名空间符号。

---

## 二、问题分析

### 问题 1：`internal` 名不副实

`src/common/` 下的 `oneq::internal::validation`、`timing`、`rcs`、`atmosphere`、`runtime` 等符号已经被多个模块直接依赖。它们不是单个模块或单个翻译单元的实现细节，而是跨模块共享设施。命名空间 `internal` 暗示“不要依赖”，但实际架构已经把它们当作公共实现层工具使用。

### 问题 2：Dual-Alias 没有价值

同一份声明在同一个头文件中出现两套路径，用 `using` 逐行同步：

- 每个文件增加样板行。
- 新增符号时容易只加 `common` 路径而忘记 alias。
- 读者需要判断该使用 `oneq::common::*` 还是 `oneq::internal::*`。
- 当前没有代码引用这些 alias，删除不改变行为。

### 问题 3：与目录结构冗余

`src/` 下的文件对库外部消费者不可见，真正公开接口在 `include/1q/` 下。`src/common/` 已经表达“内部实现层的跨模块共享设施”，再额外套 `oneq::internal::*` 会让“内部实现”与“公共共享”两个概念混在一起。

### 问题 4：命名空间层次不统一

| 文件 | 路径 | 命名空间 |
|---|---|---|
| `TimeUtils.h` | `src/common/trace/` | `oneq::internal::trace` |
| `JsonFormatUtils.h` | `src/common/trace/` | `oneq::trace::internal` |

同一个目录下的工具函数使用两种层次，会增加调用方和后续迁移成本。

---

## 三、推荐方案

### 核心原则

> `src/common/` 目录 = `oneq::common::*` 命名空间 = 跨模块共享的实现层公共设施。

`src/` 本身已经限制外部可见性；`common` 表达模块间共享边界；`internal` 不再提供额外访问控制价值。

### 步骤 1：删除 Dual-Alias（低风险，可独立执行）

涉及 5 个头文件，删除 `oneq::internal::numerics`、`oneq::internal::geometry`、`oneq::internal::coordinate_utils` alias 块：

1. `src/common/numerics/Constants.h`
2. `src/common/numerics/ClampUtils.h`
3. `src/common/numerics/NumericGuard.h`
4. `src/common/geometry/GeometryTransform.h`
5. `src/common/coordinate/CoordinateUtils.h`

验收：

```bash
rg -n "oneq::internal::(numerics|geometry|coordinate_utils)::" . \
  --glob '*.{h,hpp,cpp,cc}' \
  --glob '!build/**' \
  --glob '!third_party/**'
cmake --build build/llvm-ninja-release-local --target 1q_unit_tests -j 8
```

### 步骤 2：升级跨模块公共设施到 `oneq::common::*`

| 当前命名空间 | 目标命名空间 | 声明 / 实现文件 |
|---|---|---|
| `oneq::internal::validation::*` | `oneq::common::validation::*` | `src/common/validation/ValidationUtils.h` |
| `oneq::internal::timing::*` | `oneq::common::timing::*` | `src/common/timing/TimingRegimeModel.h`, `src/common/timing/TimingRegimeModel.cpp` |
| `oneq::internal::rcs::*` | `oneq::common::rcs::*` | `src/common/rcs/RcsPhysics.h`, `src/common/rcs/RcsPhysics.cpp` |
| `oneq::internal::atmosphere::*` | `oneq::common::atmosphere::*` | `src/common/atmosphere/StandardAtmosphere.h`, `src/common/atmosphere/StandardAtmosphere.cpp`, `src/common/atmosphere/AtmospherePhysics.h`, `src/common/atmosphere/AtmospherePhysics.cpp` |
| `oneq::internal::trace::*` | `oneq::common::trace::*` | `src/common/trace/TimeUtils.h` |
| `oneq::trace::internal::*` | `oneq::common::trace::*` | `src/common/trace/JsonFormatUtils.h` |
| `oneq::internal::output::*` | `oneq::common::output::*` | `src/common/output/OutputFrameUtils.h` |
| `oneq::internal::runtime::*` | `oneq::common::runtime::*` | `src/common/runtime/RuntimeCycleExecutor.h` |

迁移前必须用下列命令刷新调用方清单，并把对应调用方与声明文件放在同一闭环批次中：

```bash
rg -n "oneq::internal::(validation|timing|rcs|atmosphere|trace|output|runtime)|trace::internal" \
  src tests \
  --glob '*.{h,hpp,cpp,cc}' \
  --glob '!**/generated/**'
```

### 步骤 3：处理 `.cpp` 匿名 namespace（只做必要清理）

迁移 `.cpp` 时不要把匿名 namespace helper 误删。推荐形态：

```cpp
namespace oneq {
namespace common {
namespace timing {
namespace {
constexpr float kDefaultPfa = 1.0e-6f;
}  // namespace

ResolvedCycleTimingState ResolveCycleTimingState(...) { ... }

}  // namespace timing
}  // namespace common
}  // namespace oneq
```

只有当 helper 已经需要跨文件使用时，才考虑从匿名 namespace 移出；该动作不属于本轮命名空间清理的默认范围。

### 不采纳的方案

| 方案 | 理由 |
|---|---|
| 只删 dual-alias | 不解决 `src/common/` 共享设施仍挂在 `internal` 下的问题 |
| 扁平化到 `oneq::X::*` | `oneq::numerics` / `oneq::runtime` 等顶层名称容易与模块或未来公共 API 混淆，改动也更大 |
| 保留 `oneq::trace::internal` | 与 `src/common/trace/TimeUtils.h` 不一致，且 `ReplayTrace` 已经同时依赖两种 trace helper 路径 |

---

## 四、实施策略

`CLAUDE.md` 约束：**Never modify more than 5 files concurrently without intermediate build and test validation**。因此迁移要按“最多 5 个文件 + 中间构建/测试”推进，但每个批次必须是可编译闭环。

### 批次 A：删除 Dual-Alias（5 个文件）

1. `src/common/numerics/Constants.h`
2. `src/common/numerics/ClampUtils.h`
3. `src/common/numerics/NumericGuard.h`
4. `src/common/geometry/GeometryTransform.h`
5. `src/common/coordinate/CoordinateUtils.h`

验证：

```bash
cmake --build build/llvm-ninja-release-local --target 1q_unit_tests -j 8
```

### 批次 B：迁移小型命名空间族

优先处理调用方少、闭环小的族，每组不超过 5 个文件：

1. `runtime`: `RuntimeCycleExecutor.h` + AR / ESR controller 调用方。
2. `output`: `OutputFrameUtils.h` + ESR output manager + output frame 单测。
3. `trace`: `TimeUtils.h` / `JsonFormatUtils.h` + `TraceSink.cpp` / `ReplayTrace.cpp`。
4. `rcs`: `RcsPhysics.h` / `RcsPhysics.cpp` + AR propagation / detection 调用方 + RCS 单测。

每组验证：

```bash
cmake --build build/llvm-ninja-release-local --target 1q_unit_tests -j 8
```

### 批次 C：迁移 timing / atmosphere

`timing` 需要同步生产代码和测试：

- `src/common/timing/TimingRegimeModel.h`
- `src/common/timing/TimingRegimeModel.cpp`
- AR detection timing 调用方
- ESR intercept timing 调用方
- `tests/unit/ar_signal_detection_test.cpp`
- `tests/unit/ar_timing_regime_model_test.cpp`

`atmosphere` 至少涉及 4 个声明/实现文件，再叠加 AR detection 和相关单测，必须拆成不超过 5 个文件的子批，并保证每个子批之后能构建通过。

### 批次 D：迁移 validation

`validation` 是最大批次，不应作为“5 个文件示例调用方”处理。迁移前用 live `rg` 生成全量列表，当前至少覆盖：

- AR: config mapper、external input/output adapter、input validation。
- ESR: runtime config resolver、resolution rules、pipeline、external input/output adapter、input validation。
- EOS: runtime config resolver、external output adapter、input validation。
- SAR: runtime config resolver。
- common coordinate: attitude / position / velocity transform。
- 相关 validation 单测。

推荐做法：

1. 先改 `ValidationUtils.h`，并在同一批内只处理一个模块或一个目录的调用方。
2. 如果为了保持中间构建，短期保留兼容 alias，则必须在同一阶段加 TODO 和最终移除批次，避免新旧路径长期并存。
3. 每个子批后构建并至少运行对应模块 focused tests；最终批次后用 `rg "oneq::internal::validation"` 验收为零。

最终验收：

```bash
rg -n "oneq::internal::(validation|timing|rcs|atmosphere|trace|output|runtime)|trace::internal" \
  src tests \
  --glob '*.{h,hpp,cpp,cc}' \
  --glob '!**/generated/**'
cmake --build build/llvm-ninja-release-local --target 1q_unit_tests -j 8
ctest --test-dir build/llvm-ninja-release-local --output-on-failure
```

---

## 五、风险与回退

| 风险 | 缓解 |
|---|---|
| 头文件命名空间改名后仍有调用方使用旧路径 | 每个命名空间族迁移前后都运行 live `rg`；不要只依赖文档中的示例列表 |
| 单个族调用方超过 5 个文件，无法一次改完又保持构建通过 | 临时兼容 alias 只作为过渡手段使用，必须有明确移除批次；优先按模块或目录拆分 |
| `.cpp` 匿名 namespace helper 被误导出 | 只替换外层 `oneq::internal::*`，保留匿名 namespace |
| `internal` 可能在 FlatBuffers schema 中使用 | 当前 `schemas/*.fbs` 未发现 `namespace internal`；后续复核使用 `rg -n "namespace internal" schemas --glob '*.fbs'` |
| 测试代码引用生产 `oneq::internal::*` | 当前测试中存在 `oneq::internal::timing` 引用，迁移时必须同步更新；测试本地 `namespace internal` 辅助函数不属于本轮范围 |
