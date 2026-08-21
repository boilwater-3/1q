# 1Q Simulation Model Library

## Project Overview

A static library of simulation models for external service modules — airborne radar (AR), electronic surveillance radar (ESR), synthetic aperture radar (SAR), electro-optical sensor (EOS), flight dynamics, space-based infrared sensor (SBIRS), and remote identification radar (RIR) — plus electronic countermeasure (ECM), navigation, fusion, and threat assessment algorithm modules, delivered as a single linkable artifact.

## Tech Stack

- Language: C++17 (minimum C++11)
- Build: CMake, Conan
- Test: GTest/GMock
- Runtime libraries: Eigen, nanoflann, Boost, FlatBuffers, zlib, HighFive; plus spdlog/fmt and JSBSim (non-Windows only; on Windows the built-in `ProjectFileLog` backend carries `PROJECT_LOG_*` to `1q_library.log`, gated by `ONEQ_ENABLE_FILE_LOG`)

## Directory Structure

- **Modules** — `include/1q/<module>/` + `src/<module>/`: `airborne_radar` (AR), `electronic_surveillance_radar` (ESR), `sar` (SAR), `electro_optical_sensor` (EOS), `flight_dynamic`, `sbirs_sensor` (SBIRS), `remote_identification_radar` (RIR), `electronic_countermeasure` (ECM), `navigation`, `fusion`, `threat_assessment`
- **Shared** — public headers in `include/1q/<domain>/` (`coordinate`, `electromagnetics`, `environment`, `foundation`, `replay`, `trace`); shared impl in `src/common/` (`estimation`, `geometry`, `logging`, `numerics`, `output`, `rcs`, `runtime`, `timing`, `trace`, `validation`)
- **Tests** — `tests/`: `unit` (per-module), `integration`, `contract`, `performance`, `consumer`; **Examples** — `examples/`; **Tools** — `tools/` (`schemas/` — FlatBuffers definitions)
- **Build & config** — `CMakeLists.txt`, `CMakePresets.json` (shared) + `CMakeUserPresets.json` (machine-local, not in git), `conanfile.py`, `cmake/`, `.clang-format` / `.clang-tidy` / `.editorconfig`
- **Docs** — `docs/`: `common/contract.md` (cross-module contracts) + per-module design-doc sets (`design.md` + `boundaries.md` + `data-flow.md` + `algorithms.md`); build/test governance in `docs/practice/`

## Build and Test

- Bootstrap + configure are **one-time** per preset; re-run only after dependency or CMake changes.
- Build before test, serially, same preset; presets may run in parallel. Log prefix `/tmp/1q`, ctest `-j 4`.
- Verify focused: `ctest --preset <preset> -R "unit::<module>"`. Do not run full debug-local suites; full suites run on release-local only (`/completeness-review`).
- macOS mainline: `llvm-ninja-*` presets; daily dev uses `*-local` variants from per-machine `CMakeUserPresets.json`; prefer release (JSBSim ~6× faster); take install paths from the preset's actual binaryDir (`llvm-ninja-release` has no `-local` suffix).
- Windows local: `VisualStudio.15.0-amd64` (v141, multi-config). Entry is `scripts/1q.sh` (Git Bash or WSL). Environment, troubleshooting, delivery runbook: `docs/practice/windows_local_build.md`.

### macOS command flow

One-time:

```bash
bash scripts/bootstrap_conan.sh "$preset" >"${log_prefix}-conan.log" 2>&1 || { tail -n 80 "${log_prefix}-conan.log"; false; }
cmake --preset "$preset" >"${log_prefix}-cmake.log" 2>&1 || { tail -n 80 "${log_prefix}-cmake.log"; false; }
```

Daily:

```bash
cmake --build --preset "$preset" >"${log_prefix}-build.log" 2>&1 || { tail -n 80 "${log_prefix}-build.log"; false; }
ctest --preset "$preset" -R "unit::<module>" --output-on-failure -j 4
cmake --install "build/${preset}"   # packaging only
```

### Windows command flow (v141)

Use `scripts/1q.sh` from Git Bash or WSL. `doctor` must show Windows `cmake=` / `ctest=` paths. Do not point a Linux-native cmake at `VisualStudio.*` presets. Build only the needed target; BOM is auto-fixed by the pre-commit hook. After public-header layout changes, batch SEH `0xc0000005` / spurious `bad_alloc` → `scripts/1q.sh clean-stale VisualStudio.15.0-amd64 <module>`, then rebuild.

One-time:

```bash
source scripts/activate_1q_git_bash.sh
scripts/1q.sh bootstrap VisualStudio.15.0-amd64
scripts/1q.sh configure VisualStudio.15.0-amd64
```

Daily:

```bash
scripts/1q.sh build VisualStudio.15.0-amd64-release --target 1q_<module>_unit_tests
scripts/1q.sh test VisualStudio.15.0-amd64-release -R "unit::<module>"
```

### Delivery tier (VS2015 customer integration)

- Contract: C++11 + UTF-8 BOM on every tracked C/C++ file + no `/utf-8` anywhere — customers compile our headers inside their own VS2015 TUs with no extra flags.
- Maintenance chain: `scripts/utf8_bom.py` + pre-commit hook + CI `check` + `.editorconfig`.
- Presets: `1q_log_vs2015` / `VisualStudio.14.0-amd64-none`.
- Verification project: `D:\1q\1q_consumer` — zero-modification consumer mirror; builds the installed `1q.lib` as C++11 / no-`/utf-8` (VS2015 generator, simulating the customer vcxproj). Pass = zero C4819 in the build log + `[SbirsAccept]`/`[RirAccept]` in `1q_library.log`. Runbook: `docs/practice/windows_local_build.md`.

### Module and coverage switches

- `ONEQ_ENABLE_FLIGHT_DYNAMIC` (default **OFF**) gates `src/flight_dynamic/` and its tests; JSBSim is a flight_dynamic-exclusive dependency. Windows conan never installs jsbsim — with FD=ON use `ONEQ_JSBSIM_FROM_SOURCE` (third_party source) or `ONEQ_JSBSIM_PREBUILT_ROOT_DIR` (prebuilt tree).
- Coverage (macOS only): `llvm-ninja-coverage` preset + `tools/coverage_report.sh` — **never run ctest by hand** (the script owns `.profraw` placement). Branch coverage is the primary metric; see `docs/practice/coverage.md`.

## Documentation

Each module keeps a design-doc set — `design.md` (navigation entry) + `boundaries.md` + `data-flow.md` + `algorithms.md` — as its design authority: positioning, boundaries, non-goals, veto rationale, and `[evidence: ...]` annotations live there; step-by-step algorithm logic belongs in code. Update the set whenever thresholds, limitations, or veto decisions change.

## Engineering Conventions

**The project has not been delivered: refactoring does not chase excessive compatibility — clean boundaries are the first principle.**

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
- Every `PROJECT_LOG_*` call site should carry a Chinese comment above it (two-line style: `// 中译：…` + `// 标识：…`, explaining the log's meaning, trigger conditions, and state semantics for non-specialist developers); log message text stays in English.

## Session Workflow

**User work rules**
- Document clarification gate: confirm uncertain issues (direction, terminology, semantics, scope, gate values) with the user before writing — never embed silent assumptions; adjudicated content and frozen contracts need no re-confirmation.
- Plain-language bias: user-facing text favors plain language, terms explained on first use; readability outranks brevity; code comments and normative contracts keep full precision.

**Branching**
- SessionStart hook prompts on `main`; pre-commit safety net auto-creates `feature/auto-<timestamp>` on `main`/`master`.
- Naming: `feature/<short-description>` in kebab-case; merge into `main` with `--no-ff` only after user approval; delete the branch after merge.

**Commits**
- Conventional Commits: `type(scope): description` — types `feat`/`fix`/`refactor`/`test`/`docs`/`chore`/`perf`; scope = primary module; imperative, lowercase.
- End every message with `Co-Authored-By: Claude <noreply@anthropic.com>`.

**Commit gate**
- Pre-commit hook blocks `major` C++ changes (≥3 core files / ≥50 core lines) until `/completeness-review`; then retry with `ONEQ_ALLOW_MAJOR=1 git commit ...`. `minor`/`trivial` pass.

## Done Means
- The chosen preset builds successfully; relevant tests pass; the static library links cleanly into dependent targets.
- New public API or significant logic changes include new or updated tests under `tests/`.
- Documentation stays accurate: the relevant module design-doc set and its `[evidence: ...]` are updated when thresholds, limitations, or veto decisions change.
- When a plan was used, every plan item is implemented or explicitly marked as deferred.
