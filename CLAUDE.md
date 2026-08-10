# 1Q Simulation Model Library

## Project Overview

A static library of simulation models for external service modules — airborne radar (AR), electronic surveillance radar (ESR), synthetic aperture radar (SAR), electro-optical sensor (EOS), flight dynamics, and space-based infrared sensor (SBIRS) — delivered as a single linkable artifact.

## Tech Stack

- Language: C++17 (minimum C++11)
- Build: CMake, Conan
- Test: GTest/GMock
- Runtime libraries: Eigen, nanoflann, Boost, FlatBuffers, zlib, HighFive; plus spdlog/fmt and JSBSim (non-Windows only)

## Directory Structure

- **Modules** — `include/1q/<module>/` + `src/<module>/`:
  `airborne_radar` (AR) — airborne radar,
  `electronic_surveillance_radar` (ESR) — electronic surveillance radar,
  `sar` (SAR) — synthetic aperture radar,
  `electro_optical_sensor` (EOS) — electro-optical sensor,
  `flight_dynamic` — flight dynamics,
  `sbirs_sensor` (SBIRS) — space-based infrared sensor
- **Shared headers** — `include/1q/<domain>/` cross-module public types:
  `coordinate` — coordinate transforms, `environment` — environment models,
  `foundation` — base types & utilities, `replay` — replay framework,
  `trace` — telemetry trace
- **Shared impl** — `src/common/`:
  `estimation` — estimators, `geometry` — geometric ops, `logging` — log setup,
  `numerics` — numerical utils, `output` — output serialization, `rcs` — radar cross section,
  `runtime` — runtime state, `timing` — clock & timers, `trace` — trace impl,
  `validation` — input validation
- **Tests** — `tests/`:
  `unit` — per-module unit tests, `integration` — cross-module integration tests,
  `contract` — API contract tests, `performance` — benchmarks,
  `consumer` — consumer-side acceptance tests
- **Examples** — `examples/` (per-module usage demos)
- **Tools** — `tools/` (helper scripts; `schemas/` — FlatBuffers definitions)
- **Build & config** — `CMakeLists.txt` — top-level build,
  `CMakePresets.json` — shared presets, `CMakeUserPresets.json` — local presets,
  `conanfile.py` — dependency manifest, `cmake/` — cmake modules,
  `.clang-format` / `.clang-tidy` / `.editorconfig` — lint & style
- **Docs** — `docs/` (`common/contract.md` — cross-module contracts; each module keeps a design-doc set: `design.md` navigation entry + `boundaries.md` + `data-flow.md` + `algorithms.md`)

## Build and Test

- Presets: `llvm-ninja-debug-local`, `llvm-ninja-release-local` (from `CMakeUserPresets.json`; base presets in `CMakePresets.json`). Prefer release — JSBSim runs ~6× faster.
- Run bootstrap → configure → build → test **serially** for the same preset; parallel only across different presets.
- Log prefix `/tmp/1q`, ctest `-j 4`.

```bash
bash scripts/bootstrap_conan.sh "$preset" >"${log_prefix}-conan.log" 2>&1 || { tail -n 80 "${log_prefix}-conan.log"; false; }
cmake --preset "$preset" >"${log_prefix}-cmake.log" 2>&1 || { tail -n 80 "${log_prefix}-cmake.log"; false; }
cmake --build --preset "$preset" >"${log_prefix}-build.log" 2>&1 || { tail -n 80 "${log_prefix}-build.log"; false; }
ctest --preset "$preset" --output-on-failure -j 4
```

- `ONEQ_ENABLE_FLIGHT_DYNAMIC` (default **OFF**) gates `src/flight_dynamic/` and its tests. JSBSim remains required regardless (`src/common/environment/JsbsimAtmosphereAdapter`).
- Code coverage: `llvm-ninja-coverage` preset + `tools/coverage_report.sh`. **Never run ctest by hand** — the script owns `.profraw` placement. Branch coverage is the primary metric; see `docs/practice/coverage.md`.

## Documentation

Each module keeps a design-doc set with `design.md` as the navigation entry and `boundaries.md`, `data-flow.md`, `algorithms.md` as its content. Together they are the design authority for the module:
- `design.md` — module positioning, mental model, and navigation to the other three.
- `boundaries.md` — module-level boundaries, non-goals, veto decisions with quantified thresholds, and change rules.
- `data-flow.md` — architecture diagrams (mermaid component/sequence/data-flow), I/O, and state ownership.
- `algorithms.md` — algorithm registry table + per-algorithm implementation boundaries, counter-intuitive points, and `[evidence: ...]` annotations.

Docs capture what code alone cannot convey (positioning, boundaries, non-goals, counter-intuitive behavior, veto rationale); step-by-step algorithm logic belongs in code.

## Engineering Conventions

**Style**
- Follow the Google C++ Style Guide.
- Keep namespace-directory mapping consistent.
- Keep changes in `src/` unless a public API change is unavoidable.
- Mark variables and member functions `const` by default; relax only when mutation is required.
- Never introduce C++ exceptions.
- Never reformat existing code that was not touched by the current change.

**Architecture**
- Prefer abstract interfaces at module boundaries.
- Prefer forward declarations to reduce includes and rebuild cost.
- Use PIMPL for critical classes when hiding implementation reduces recompilation propagation.
- Make interfaces easy to use correctly and hard to use incorrectly.
- Guard against known edge cases, not hypothetical ones — don't over-defend at boundaries.

**Process**
- Automated bulk edits: verify the exact command on 1-2 files first, then proceed in batches of ≤5 files.
- Semantic refactors: commit as the smallest buildable and testable closure; enumerate the dependency closure and validate at real compileable boundaries.

**Logging**
- Log critical actions and failures using the project's logging facility, when available.
- 每个 `PROJECT_LOG_*` 调用点上方应有中文注释（`// 中译：…` + `// 标识：…` 两行式，
  面向非专业开发人员解释日志含义、触发条件与状态语义）；日志消息文本保持英文。

## Session Workflow

- **Plan mode & branching**: SessionStart hook prompts when on `main`; pre-commit hook auto-creates `feature/<topic>` from commit message on `main`/`master` as a safety net. Branch naming: `feature/<short-description>` in kebab-case.
- **Commit messages**: [Conventional Commits](https://www.conventionalcommits.org/) format — `type(scope): description`. Types: `feat`, `fix`, `refactor`, `test`, `docs`, `chore`, `perf`. Scope is the primary module/domain (e.g., `airborne_radar`, `eos`, `sar`). Description in imperative mood, lowercase. End every message with `Co-Authored-By: Claude <noreply@anthropic.com>`.
- **Commit gate**: pre-commit hook blocks `major` C++ changes (≥3 files or ≥50 lines) until `/completeness-review` passes. `minor` and `trivial` changes pass through with a warning.
- **Merge & cleanup**: merge the feature branch into `main` with `--no-ff` only after user approval; delete it locally (and on remote if pushed) to avoid branch proliferation.

## Done Means
- The chosen preset builds successfully.
- Relevant tests pass for the chosen preset.
- The static library links cleanly into dependent targets.
- New public API or significant logic changes include new or updated tests under `tests/`.
- Documentation stays accurate: update the relevant module design-doc set (`design.md`/`boundaries.md`/`data-flow.md`/`algorithms.md`) and its `[evidence: ...]` when thresholds, limitations, or veto decisions change.
- When a plan was used, every plan item is implemented or explicitly marked as deferred.
