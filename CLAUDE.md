# 1Q Simulation Model Library

## Project Overview
Simulation model library for external service modules: airborne radar (AR), electronic surveillance radar (ESR), synthetic aperture radar (SAR), electro-optical sensor (EOS), flight dynamics, and space-based infrared sensor (SBIRS).

## Tech Stack
C++17 (minimum C++11), CMake, Conan, GTest/GMock, Eigen, nanoflann, Boost, FlatBuffers, zlib, spdlog/fmt, JSBSim, HighFive (optional).

## Directory Structure
Module business domains (each has a matching `include/1q/<module>/` public API and `src/<module>/` implementation):
- `airborne_radar` (AR), `electronic_surveillance_radar` (ESR), `sar` (SAR), `electro_optical_sensor` (EOS), `flight_dynamic`, `sbirs_sensor` (SBIRS)
- Cross-domain shared types under `include/1q/{coordinate,environment,foundation,replay,trace}/`
- Cross-module shared implementation under `src/common/` (estimation, geometry, logging, numerics, output, rcs, runtime, timing, trace, validation, ...)

Other top-level:
- `tests/` — unit / integration / contract / performance / consumer
- `examples/` — per-module usage examples
- `tools/` — helper scripts (`schemas/` holds FlatBuffers definitions)
- Root: `CMakeLists.txt`, `CMakePresets.json`, `CMakeUserPresets.json`, `conanfile.py`, `cmake/`, plus lint configs (`.clang-format`, `.clang-tidy`, `.editorconfig`)

Use `ls`/`find` for sub-directory detail. Namespace-directory mapping is consistent — see Engineering Conventions.

## Build and Test

- Presets: `llvm-ninja-debug-local`, `llvm-ninja-release-local` (from `CMakeUserPresets.json`; base presets `llvm-ninja-debug`/`llvm-ninja-release` live in `CMakePresets.json`). Prefer release for testing — JSBSim runs ~6× faster.
- Run bootstrap → configure → build → test **serially** for the same preset; never start ctest before build completes. Parallel work is allowed only across different presets.
- Use log prefix `/tmp/1q` and `-j 4` for parallel test execution.

```bash
bash scripts/bootstrap_conan.sh "$preset" >"${log_prefix}-conan.log" 2>&1 || { tail -n 80 "${log_prefix}-conan.log"; false; }
cmake --preset "$preset" >"${log_prefix}-cmake.log" 2>&1 || { tail -n 80 "${log_prefix}-cmake.log"; false; }
cmake --build --preset "$preset" >"${log_prefix}-build.log" 2>&1 || { tail -n 80 "${log_prefix}-build.log"; false; }
ctest --preset "$preset" --output-on-failure -j 4
```

### flight_dynamic module build switch

`ONEQ_ENABLE_FLIGHT_DYNAMIC` (default **OFF**) gates `src/flight_dynamic/`. When OFF, all its targets, the `1q_fd_tests` binary, both CTest partitions (`unit::flight_dynamic` and `known_limit::flight_dynamic`), and examples are skipped.

```bash
cmake --preset "$preset" -D ONEQ_ENABLE_FLIGHT_DYNAMIC=ON
cmake --build --preset "$preset"
ctest --preset "$preset" -R 'flight_dynamic'          # stable + known_limit
```

Caveats: disabling it does **not** drop JSBSim (`src/common/environment/JsbsimAtmosphereAdapter` still needs it); `fd_*` sources are always excluded from `1q_unit_tests`; no preset sets this option today.

### Code coverage

- `llvm-ninja-coverage` preset + `tools/coverage_report.sh` own the full flow (build → ctest → profraw → profdata → report).
- **Never run `ctest` by hand for coverage** — it scatters `.profraw`; always invoke the script so it owns profile placement.
- Common options: `--label <ctest-label>`, `--clean` (wipe before regen), `--open`, `--no-test` (regen from existing profraw).
- Read coverage as a diagnostic, not a KPI. **Branch coverage is the primary metric** — line coverage can read 100% while whole `else` branches stay untested.
- See `docs/practice/coverage.md` for metric definitions, report reading, troubleshooting, and baseline numbers.

## Documentation

Design documentation lives in `docs/` and follows a minimal structure:

- `docs/common/contract.md` — cross-module contracts, public API rules, output model, document governance
- One `design.md` per module — architecture, algorithms, data flow, limitations with inline `[evidence: ...]` references

Each `design.md` is the sole design authority for its module. It contains:
- Architecture overview (mermaid component/sequence/data-flow diagrams)
- Algorithm descriptions with test-backed `[evidence: ...]` inline annotations
- Limitations and veto decisions with quantified thresholds

Evidence references point to existing test files and specific test cases (not branch paths or external docs).

## Engineering Conventions
- Follow the Google C++ Style Guide.
- Keep namespace-directory mapping consistent.
- Prefer internal `src/` changes before widening public headers under `include/`.
- Prefer abstract interfaces at module boundaries.
- Prefer forward declarations to reduce includes and rebuild cost.
- Use PIMPL for critical classes when hiding implementation reduces recompilation propagation.
- Make interfaces easy to use correctly and hard to use incorrectly.
- For algorithm, architecture, module-internal optimization, output/config semantics, or public API work, use `skills/evidence-first-freeze-contract` before implementation.
- Log critical paths and events. e.g. Use `spdlog::debug/info` for flow and `spdlog::error` for failures.
- For Chinese Doxygen work, invoke the `/cpp-chinese-doxygen` skill.

## Constraints
- Never introduce C++ exceptions.
- Never introduce project-specific identifiers or prefixes in `cmake/`.
- Never reformat existing code that was not touched by the current change.

## Batch Refactoring Safety

- Never run terminal commands (e.g., `sed`, `awk`, scripts) or perform automated bulk edits without importantly verifying the exact command on 1-2 files first.
- Never modify more than 5 files concurrently without intermediate build and test validation to prevent mass codebase corruption and cascading repair cycles.

## Done Means
- The chosen preset builds successfully.
- Relevant tests pass for the chosen preset.
- New public API or significant logic changes include new or updated tests under `tests/`.
- Documentation stays accurate for changed behavior: update the relevant `design.md` and its inline `[evidence: ...]` references if thresholds, limitations, or veto decisions change.

note: Your mileage may vary. Not all of these rules are necessarily optimal for every setup. Like anything else, feel free to break the rules once...
- you understand when & why it's okay to break them.
- you have a good reason to do so.
