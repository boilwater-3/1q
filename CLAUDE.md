# AGENTS.md

## Project Overview
Simulation model library for external service modules such as airborne radar and electronic surveillance radar.

## Tech Stack
C++11, CMake, Conan, GTest/GMock, spdlog, Eigen, nanoflann.

## Directory Structure
```text
include/
`-- 1q/
    |-- airborne_radar/                     airborne radar public API surface
    |-- electro_optical_sensor/             electro optical sensor public API surface
    |-- electronic_surveillance_radar/      ESR public API surface
    |-- foundation/                         cross-domain public foundation types
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
`-- electronic_surveillance_radar/
    |-- environment/                        ESR environment modeling and state
    |-- intercept/                          ESR signal/intercept domain logic
    |-- model/                              ESR model helpers
    |-- output/                             ESR output manager and formatters
    |-- pipeline/                           ESR pipeline composition and processing
    |-- runtime/                            ESR controller and runtime telemetry
    |-- session/                            ESR session composition/config/runtime resolvers
    `-- utils/                              ESR utility helpers
tests/                              unit and integration tests
examples/                           usage examples
tools/                              helper scripts
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
- Use stress preset only for stress runs: `llvm-ninja-release-local-stress`.
- Use log prefix: `/tmp/1q`.
- Run configure, build, and test serially for the same preset.
- Never start ctest before that preset build completes.
- Parallel work is allowed only across different presets.

```bash
cmake --preset "$preset" >"${log_prefix}-cmake.log" 2>&1 || { tail -n 80 "${log_prefix}-cmake.log"; false; }
cmake --build --preset "$preset" >"${log_prefix}-build.log" 2>&1 || { tail -n 80 "${log_prefix}-build.log"; false; }
ctest --preset "$preset" --output-on-failure
```

## Engineering Conventions
- Follow the Google C++ Style Guide.
- Keep namespace-directory mapping consistent.
- Prefer internal `src/` changes before widening public headers under `include/`.
- Prefer abstract interfaces at module boundaries.
- Prefer forward declarations to reduce includes and rebuild cost.
- Use PIMPL for critical classes when hiding implementation reduces recompilation propagation.
- Make interfaces easy to use correctly and hard to use incorrectly.
- Log critical paths and events.e.g Use `spdlog::debug/info` for flow and `spdlog::error` for failures.
- For Chinese Doxygen work, explicitly use `$cpp-chinese-doxygen`.
- 

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
- Documentation stays accurate for changed behavior.

note: Your mileage may vary. Not all of these rules are necessarily optimal for every setup. Like anything else, feel free to break the rules once...
- you understand when & why it's okay to break them.
- you have a good reason to do so.
