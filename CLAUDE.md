# 1Q Simulation Model Library

## Project Overview
Simulation model library for external service modules such as airborne radar and electronic surveillance radar.

## Tech Stack
C++11, CMake, Conan, GTest/GMock, spdlog, Eigen, nanoflann.

## Directory Structure
```text
include/
`-- 1q/
    |-- airborne_radar/                     airborne radar public API surface
    |-- coordinate/                         shared coordinate & kinematics reference types
    |-- electro_optical_sensor/             electro optical sensor public API surface
    |-- electronic_surveillance_radar/      ESR public API surface
    |-- environment/                        unified environment modeling (atmosphere/observers)
    |-- flight_dynamic/                     flight dynamics & maneuver public API surface
    |-- foundation/                         cross-domain public foundation types
    |-- replay/                             replay public interfaces
    |-- sar/                                SAR public API surface
    `-- trace/                              cross-domain trace interfaces
src/
|-- airborne_radar/
|   |-- decision/                           decision pipeline and control orchestration
|   |-- environment/                        environment modeling and state
|   |-- model/                              domain model mapping and helpers
|   |-- output/                             output assembly and query helpers
|   |-- runtime/                            cycle orchestration and runtime control
|   |-- session/                            session composition/config/runtime resolvers
|   |-- signal/                             detection, association, tracking, and signal pipeline
|   `-- utils/                              airborne utility adapters
|-- common/                                 shared implementation utilities
|   |-- atmosphere/                         atmosphere and propagation shared physics
|   |-- geometry/                           coordinate and geometry transforms
|   |-- logging/                            project logging abstraction
|   |-- numerics/                           numerical solvers and math helpers
|   |-- output/                             shared output shaping helpers
|   |-- rcs/                                radar cross section physics helpers
|   |-- runtime/                            runtime shared facilities
|   |-- timing/                             timing regime models
|   |-- trace/                              trace sink and adapters
|   `-- validation/                         shared validation helpers
|-- electro_optical_sensor/
|   |-- environment/                        EOS environment modeling and state
|   |-- foundation/                         EOS physics and optical foundation modules
|   |-- model/                              EOS model/input validation
|   |-- runtime/                            EOS controller and orchestration runtime
|   |-- session/                            EOS session composition/runtime config
|   |-- signal/                             EOS signal pipeline
|   `-- utils/                              EOS utilities
|-- electronic_surveillance_radar/
|   |-- environment/                        ESR environment modeling and state
|   |-- intercept/                          ESR signal/intercept domain logic
|   |-- model/                              ESR model helpers
|   |-- output/                             ESR output manager and formatters
|   |-- pipeline/                           ESR pipeline composition and processing
|   |-- runtime/                            ESR controller and runtime telemetry
|   |-- session/                            ESR session composition/config/runtime resolvers
|   `-- utils/                              ESR utility helpers
|-- flight_dynamic/
|   |-- adapter/                            JSBSim FDM adapter (property-tree bridge)
|   |-- autopilot/                          multi-channel autopilot (heading/alt/speed/pitch/roll)
|   |-- guidance/                           maneuver executor FSM and waypoint sequencing
|   |-- model/                              vehicle state mapping from JSBSim
|   `-- propulsion/                         engine and throttle management
`-- sar/
    |-- calibration/                        radiometric calibration
    |-- echo/                               point target raw echo generation
    |-- geometry/                           L1/L2/L3 trajectory, spotlight/scansar geometry
    |-- imaging/                            RDA, BP/GBP, MoCo, Omega-K, quality, multilook
    |-- output/                             binary/sidecar/HDF5 output
    |-- runtime/                            PulseRingBuffer and aperture stitching
    |-- session/                            session assembly, input validation, imaging executor
    |-- signal/                             FFT, LFM waveform, matched filter
    `-- smoke/                              PGA toolchain compile/link smoke test
tests/                              unit and integration tests
examples/                           usage examples
tools/                              helper scripts
|-- schemas/                        FlatBuffers schema definitions
root config files/
|-- .clang-format                   C/C++ formatting rules
|-- .clang-tidy                     clang-tidy check configuration
|-- .editorconfig                   editor baseline settings
|-- .gitattributes                  repository line-ending and binary rules
|-- .gitignore                      ignored path patterns
|-- CMakeLists.txt                  top-level CMake entry point
|-- CMakePresets.json               shared configure/build/test presets
|-- CMakeUserPresets.json           local user presets for LLVM/Conan flows
|-- conanfile.py                    Conan dependency recipe
`-- cmake/                          shared CMake modules and toolchain configs
```

## Build and Test

- Use presets: `llvm-ninja-debug-local`, `llvm-ninja-release-local`.
- Prefer `llvm-ninja-release-local` for testing — JSBSim simulation runs ~6× faster than debug.
- Use stress preset only for stress runs: `llvm-ninja-release-local-stress`.
- Use log prefix: `/tmp/1q`.
- Run configure, build, and test serially for the same preset.
- Never start ctest before that preset build completes.
- Parallel work is allowed only across different presets.
- Use the `-j <num_cores>` or `--parallel <num_cores>` option (e.g., `-j 4`) to run tests in parallel, leveraging multi-core CPUs to significantly reduce test execution time.

```bash
cmake --preset "$preset" >"${log_prefix}-cmake.log" 2>&1 || { tail -n 80 "${log_prefix}-cmake.log"; false; }
cmake --build --preset "$preset" >"${log_prefix}-build.log" 2>&1 || { tail -n 80 "${log_prefix}-build.log"; false; }
ctest --preset "$preset" --output-on-failure -j 4
```

### flight_dynamic module build switch

The maneuver module (`src/flight_dynamic/`) is gated behind a CMake option:

- `ONEQ_ENABLE_FLIGHT_DYNAMIC` (default **OFF**) — when OFF, the module is excluded from compilation: `fd_engine`/`fd_core` OBJECT targets, the `1q_fd_tests` binary, its five CTest tiers (`fd_smoke`/`fd_contract`/`fd_controllability`/`fd_performance`/`fd_known_limit`), and the `flight_dynamic` examples are all skipped.

The option defaults to OFF because the module is not yet in a stable testing stage. To build or test it, pass it explicitly:

```bash
cmake --preset "$preset" -D ONEQ_ENABLE_FLIGHT_DYNAMIC=ON
cmake --build --preset "$preset"
ctest --preset "$preset" -L fd_ci        # CI subset (smoke + contract + controllability)
```

Caveats:
- Disabling this module does **not** drop the JSBSim dependency — `1q::core` still links JSBSim because `src/common/environment/JsbsimAtmosphereAdapter` uses it. Dropping JSBSim entirely is a separate decision.
- `fd_*` unit test sources are always removed from `1q_unit_tests` regardless of this option, so the module's tests never leak into the generic unit-test binary.
- No `CMakePresets.json` preset sets this option today, so every preset inherits the OFF default. If CI or a release needs the module, the preset must set `ONEQ_ENABLE_FLIGHT_DYNAMIC=ON` explicitly.

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
- For Chinese Doxygen work, explicitly use `$cpp-chinese-doxygen`.

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
