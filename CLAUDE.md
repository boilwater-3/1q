# 1Q Simulation Model Library

## Project Overview

A static library of simulation models for external service modules — airborne radar (AR), electronic surveillance radar (ESR), synthetic aperture radar (SAR), electro-optical sensor (EOS), flight dynamics, space-based infrared sensor (SBIRS), and remote identification radar (RIR) — plus electronic countermeasure (ECM), navigation, fusion, and threat assessment algorithm modules, delivered as a single linkable artifact.

## Tech Stack

- Language: C++17 (minimum C++11)
- Build: CMake, Conan
- Test: GTest/GMock
- Runtime libraries: Eigen, nanoflann, Boost, FlatBuffers, zlib, HighFive; plus spdlog/fmt and JSBSim (non-Windows only; on Windows the built-in `ProjectFileLog` backend carries `PROJECT_LOG_*` to `1q_library.log`, gated by `ONEQ_ENABLE_FILE_LOG`)

## Directory Structure

- **Modules** — `include/1q/<module>/` + `src/<module>/`:
  `airborne_radar` (AR) — airborne radar,
  `electronic_surveillance_radar` (ESR) — electronic surveillance radar,
  `sar` (SAR) — synthetic aperture radar,
  `electro_optical_sensor` (EOS) — electro-optical sensor,
  `flight_dynamic` — flight dynamics,
  `sbirs_sensor` (SBIRS) — space-based infrared sensor,
  `remote_identification_radar` (RIR) — remote identification radar,
  `electronic_countermeasure` (ECM) — electronic countermeasure,
  `navigation` — area coverage planning,
  `fusion` — multi-source fusion,
  `threat_assessment` — target threat evaluation
- **Shared headers** — `include/1q/<domain>/` cross-module public types:
  `coordinate` — coordinate transforms, `electromagnetics` — RF scene & link budget,
  `environment` — environment models, `foundation` — base types & utilities,
  `replay` — replay framework, `trace` — telemetry trace
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

- Run bootstrap → configure → build → test **serially** for the same preset; parallel only across different presets. Log prefix `/tmp/1q`, ctest `-j 4`.
- **Verification scope**: for day-to-day incremental verification, focus `ctest --preset <preset> -R <focused scope>` on a module/partition (e.g., `-R "unit::<module>"`); **do not run the full debug-local ctest suite** (slow and duplicates release); full-suite verification runs on release-local only (e.g., during `/completeness-review`).

### Platform Difference Overview (macOS mainline vs Windows v141)

macOS is the development/CI mainline (full Conan dependency set); the local Windows machine uses the legacy v141 toolset (trimmed dependencies + built-in file logging). Presets, generators, and dependency sets differ across the two platforms:

| Aspect | macOS (mainline) | Windows (local v141) |
|---|---|---|
| preset | `llvm-ninja-debug(-local)` / `llvm-ninja-release(-local)` / `llvm-ninja-coverage` | `VisualStudio.15.0-amd64` |
| Generator | Ninja, single-config (build_type fixed per preset) | VS2026 generator + v141 (14.16) toolset, multi-config (`--config Debug / Release` selects the flavor) |
| conan install | Once per preset, single build_type | bootstrap installs both Debug + Release in one pass |
| Logging | spdlog/fmt (Conan) | No third-party logger; built-in `ProjectFileLog` (gated by `ONEQ_ENABLE_FILE_LOG`, writes `1q_library.log`); the `component_attachment` example layer carries its own std::ofstream file backend (`CA_LOG_BACKEND_SPDLOG=0`) |
| HighFive / JSBSim | Installed as Conan prebuilts | Neither installed; with FD=ON, JSBSim needs third_party source or a prebuilt tree |
| Coverage | `llvm-ninja-coverage` | None |

Daily development uses the `*-local` variants (machine-local `CMakeUserPresets.json`, **not in git**; each machine keeps its own); CI uses the base presets in `CMakePresets.json`. Trap: the repo preset `llvm-ninja-release` has binaryDir `build/llvm-ninja-release` (no `-local` suffix — a historical override), while `llvm-ninja-debug` points at `build/llvm-ninja-debug-local`; take install paths from the actual binaryDir. Prefer release — JSBSim runs ~6× faster.

### macOS command flow

```bash
bash scripts/bootstrap_conan.sh "$preset" >"${log_prefix}-conan.log" 2>&1 || { tail -n 80 "${log_prefix}-conan.log"; false; }
cmake --preset "$preset" >"${log_prefix}-cmake.log" 2>&1 || { tail -n 80 "${log_prefix}-cmake.log"; false; }
cmake --build --preset "$preset" >"${log_prefix}-build.log" 2>&1 || { tail -n 80 "${log_prefix}-build.log"; false; }
ctest --preset "$preset" --output-on-failure -j 4
cmake --install "build/${preset}"   # installs to build/install/<preset>: include/1q + lib + cmake config
```

### Windows command flow (v141 mainline; Git Bash throughout)

**Git Bash session init** (once per new terminal, or persist in `~/.bashrc`):

```bash
source scripts/activate_1q_git_bash.sh
# One-time persistence (adjust repo path for this machine):
#   cp scripts/1q_env.local.sh.example scripts/1q_env.local.sh   # set ONEQ_CMAKE_ROOT
#   echo 'source "/d/1q/1q/scripts/activate_1q_git_bash.sh" 2>/dev/null' >> ~/.bashrc
```

`activate` adds `cmake`/`ctest` to PATH (including common roots such as `D:/environment/CMake`), injects `UCRTContentRoot`, and prepends `scripts/bin/cmake` + `scripts/bin/ctest` so bare `build/VisualStudio.15.0-amd64` builds are blocked and `ctest.exe` resolves even when the shell does not map `.exe` names automatically.

**Preferred unified entry** (avoids repeating the same UCRT/PATH footguns across sessions):

```bash
scripts/1q.sh bootstrap VisualStudio.15.0-amd64
scripts/1q.sh configure VisualStudio.15.0-amd64
scripts/1q.sh build VisualStudio.15.0-amd64-release --target 1q_remote_identification_radar_unit_tests
scripts/1q.sh test VisualStudio.15.0-amd64-release -R "unit::remote_identification_radar" -j 4
scripts/1q.sh doctor   # check cmake / ctest / UCRT / PATH / shell
```

#### Agent / Cursor Windows build–test gate (no thrashing)

On this machine, Cursor’s default `bash` is often **WSL / system32 bash**, not Git Bash (MINGW).
The wrong shell looks like “cmake works, ctest is NOT FOUND, UCRT flickers, build/test loops forever.”
**Before any Windows build/test, pass the gate below. If the gate fails, fix the shell/PATH — do not change product code hoping it will help.**

1. **Shell identity**: `uname -s` must be `MINGW*` / `MSYS*` (Git Bash). If it is `Linux` and paths are under `/mnt/d/...`, you are on WSL — switch to a Git Bash terminal, or set Cursor’s default profile to Git Bash.
2. **Env inject**: `source scripts/activate_1q_git_bash.sh` (once per new terminal).
3. **doctor green** (all must hold before build/test):
   - `cmake=` points at `scripts/bin/cmake` or a real cmake, and `cmake --version` runs
   - `ctest=` is **not** `NOT FOUND` (`1q.sh test` resolves `ctest.exe`; doctor must print a path)
   - `UCRTContentRoot=` is set (under MINGW)
   - no `SHELL_WARNING=Cursor/WSL bash detected...`
4. **Commands only via preset / 1q.sh** (serial: same preset, build then test):
   - `scripts/1q.sh build VisualStudio.15.0-amd64-release --target <tests>`
   - `scripts/1q.sh test VisualStudio.15.0-amd64-release -R "unit::<module>" -j 4`
5. **UTF-8 BOM (sources with Chinese comments)**: new/rewritten `.h/.cpp` that contain Chinese comments must have a BOM.
   - `scripts/utf8_bom.py convert` **only touches git-tracked files** → `git add` new files first, then `convert`, or confirm the file starts with `EF BB BF` right after write.
   - **Do not** “fix” C4819 by deleting Chinese comments; the root cause is a missing BOM.
6. **Stale artifacts / file locks**: SEH `0xc0000005` or unrelated `bad_alloc` → `clean-stale` then rebuild; `Permission denied` writing `.obj` → unlock the IDE/AV hold first; do not treat as a logic bug.
7. **Failure attribution order** (avoid thrashing): shell/PATH → doctor → BOM → stale/locks → then the compile error itself.

Canonical short rule: `.cursor/rules/windows-git-bash-build.mdc`.

Equivalent raw commands (only after `source activate`):

```bash
bash scripts/bootstrap_conan.sh VisualStudio.15.0-amd64
cmake --preset VisualStudio.15.0-amd64
cmake --build --preset VisualStudio.15.0-amd64-release
ctest --preset VisualStudio.15.0-amd64-release -R "unit::<module>" --output-on-failure -j 4
cmake --install build/VisualStudio.15.0-amd64 --config Release
```

**Stale incremental artifacts** (after public-header layout changes, if Debug/Release tests batch-fail with SEH `0xc0000005` / unrelated `bad_alloc`, clean the module then rebuild):

```bash
scripts/1q.sh clean-stale VisualStudio.15.0-amd64 remote_identification_radar
scripts/1q.sh build VisualStudio.15.0-amd64-release --target 1q_remote_identification_radar_unit_tests
```

The final library artifact lands at `build/VisualStudio.15.0-amd64/Release/lib/1q.lib`. **Builds must go through a build preset**: on this machine, the 64-bit registry value `KitsRoot10` under `HKLM\SOFTWARE\Microsoft\Windows Kits\Installed Roots` is written as the non-existent `C:\Program Files\Windows Kits\10\`, so v141's ucrt.props resolves UCRT from the registry into a dead path. A raw-directory build without the `UCRTContentRoot` environment variable (`cmake --build build/VisualStudio.15.0-amd64 ...`, reproduced 2026-08-19) fails compiling TUs with `corecrt.h` not found (C1083) and fails linking with LNK1104 (ucrtd.lib). The preset-injected environment variable makes ucrt.props prefer the env var over the registry. If you must build via the raw directory / IDE MSBuild, initialize the v141 environment first and let MSBuild consume environment paths (`/p:UseEnv=true`):

  ```bat
  call "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvarsall.bat" x64 -vcvars_ver=14.16
  cmake --build build/VisualStudio.15.0-amd64 --target <target> --config Debug -- -p:UseEnv=true -m -v:m -nologo
  ```

  Note (2026-08-17 erratum): in this environment (MSVC v141 Debug CRT), `SbirsExclusionCauseRecorderTest` failing wholesale with bad_alloc was once recorded as a "known baseline failure" — it turned out to be **stale build artifacts**: after a header-layout change, incremental Unity builds did not rebuild all TUs, and Debug hardened checks surfaced SEH crashes; deleting `build/.../src/sbirs_sensor` artifacts to force a full rebuild turned all 203 sbirs unit tests green. If Debug tests hit SEH 0xc0000005/bad_alloc unrelated to any code logic, do a full rebuild before investigating. `integration::airborne_radar` 0xc0000409 remains a separate pre-existing issue.

### VS2015 delivery tier (customer integration)

**Why this tier exists**: the customer's VS2015 (v140, any Update) environment compiles our headers inside their own TUs and cannot be told to add compiler flags. BOM-less UTF-8 sources are read as system codepage (GBK), corrupting Chinese comments. Delivery therefore = C++11 + UTF-8 BOM on every tracked C/C++ file + **no `/utf-8`** anywhere (not even INTERFACE-propagated). The BOM makes cl.exe decode UTF-8 unconditionally; `scripts/utf8_bom.py` maintains it (convert/check/strip), CI's guard job enforces `check`, `.editorconfig` declares `utf-8-bom` for C/C++.

**Preset selection**:

| Preset | Purpose / problem solved |
|---|---|
| `1q_log_vs2015` | **Delivery**: v140 x64, no Conan (fetch_third_party.bat first), C++11, SBIRS/RIR acceptance logs ON |
| `VisualStudio.14.0-amd64-none` | Delivery base (same minus acceptance logs) |
| `VisualStudio.15.0-amd64` | Windows dev mainline: VS2026 + v141, C++17, Conan |
| `llvm-ninja-*` (macOS) | CI/dev mainline: full Conan deps, spdlog, coverage |
| `VisualStudio.14.0-amd64` | VS2015 + Conan — not viable here (CMake 4.3.1 vs v140 toolset; see `docs/practice/build_and_test_governance.md`) |

**Delivery verification workflow** (end-to-end, ~15 min):

1. `cmake --preset 1q_log_vs2015 && cmake --build --preset 1q_log_vs2015-release`
2. `cmake --install build/1q_log_vs2015 --config Release`
3. Sync install → consumer (`D:\1q\1q_consumer`): `include/1q/*`, `lib/1q.lib`, and repo `examples/` → consumer `src/` (BOM'd zero-modification mirror)
4. Consumer builds **C++11, no `/utf-8`** (simulates customer vcxproj): `cmake -G "Visual Studio 14 2015" -A x64 && cmake --build --config Release`
5. Run demo 3 cycles; verify zero C4819 in build log, `[SbirsAccept]`/`[RirAccept]` present in `1q_library.log`

### Module and coverage switches

- `ONEQ_ENABLE_FLIGHT_DYNAMIC` (default **OFF**) gates `src/flight_dynamic/` and its tests; JSBSim is a flight_dynamic-exclusive CMake dependency (with FD=OFF, `JsbsimProvider` short-circuits and core/common carries no JSBSim dependency — the old note "`src/common/environment/JsbsimAtmosphereAdapter` makes JSBSim unconditionally required" is void; that file no longer exists). macOS conan installs the jsbsim package regardless of the switch (it is simply never `find_package`d); Windows conan never installs jsbsim — with FD=ON use `ONEQ_JSBSIM_FROM_SOURCE` (third_party/jsbsim source) or `ONEQ_JSBSIM_PREBUILT_ROOT_DIR` (prebuilt tree).
- Code coverage (macOS only): `llvm-ninja-coverage` preset + `tools/coverage_report.sh`. **Never run ctest by hand** — the script owns `.profraw` placement. Branch coverage is the primary metric; see `docs/practice/coverage.md`.

## Documentation

Each module keeps a design-doc set with `design.md` as the navigation entry and `boundaries.md`, `data-flow.md`, `algorithms.md` as its content. Together they are the design authority for the module:
- `design.md` — module positioning, mental model, and navigation to the other three.
- `boundaries.md` — module-level boundaries, non-goals, veto decisions with quantified thresholds, and change rules.
- `data-flow.md` — architecture diagrams (mermaid component/sequence/data-flow), I/O, and state ownership.
- `algorithms.md` — algorithm registry table + per-algorithm implementation boundaries, counter-intuitive points, and `[evidence: ...]` annotations.

Docs capture what code alone cannot convey (positioning, boundaries, non-goals, counter-intuitive behavior, veto rationale); step-by-step algorithm logic belongs in code.

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

- **Document-writing clarification gate (user work rule, 2026-08-18)**: when writing or revising any document, any uncertain or unconfirmed issue (adjudication direction, terminology alignment, field semantics, scope trade-offs, acceptance-gate values, etc.) must be **confirmed with the user before writing**; never embed assumptions into a document and leave them there — users will hardly notice assumptions buried in docs. Content the user has already adjudicated, or frozen contracts, proceeds as usual without re-confirmation.
- **Non-specialist wording bias (user work rule, 2026-08-18)**: the user is not a professional developer — user-facing descriptions (conversation replies, summaries and reporting, human-oriented doc paragraphs) should favor plain language and analogies, explaining terms on first use; do not sacrifice technical accuracy, but readability outranks brevity. Terminology precision inside library code comments and normative contract documents is unaffected by this rule.
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
