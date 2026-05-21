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
- Do not start ctest before that preset build completes.
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

- Planning files live under `plan/<plan-name>/`, not tracked by git.

## Constraints
- Do not introduce C++ exceptions.
- Do not introduce project-specific identifiers or prefixes in `cmake/`.
- Do not reformat existing code that was not touched by the current change.

## Batch Refactoring Safety Rules

These rules exist because of repeated failures during cross-file C++ refactoring (2026-05-21). perl/sed on C++ is fragile; the failures were systematic, not one-off.

### 1. Incremental validation is mandatory

```
3-5 files → edit → cmake --build → ctest → commit → next batch
```

Never batch-edit 15+ files in one shot without intermediate builds. Every broken build spawns cascading fixes that themselves may break more things. The maximum safe batch size is the number of files you can hold in your head while reading compiler errors.

### 2. Never use perl/sed `-ne 'next if /pattern/; print'` on C++ source

This is the "delete matching lines" pattern. It will destroy function implementations whose signature happens to match a pattern intended only for declarations or calls. We lost an entire 160-line GeometryTransform.cpp this way. Use the Edit tool with exact old_string/new_string for deletions instead.

### 3. Regex must distinguish function definitions from call sites

A pattern like `s/ClampFloat\(/ns::Clamp\(/g` matches both:
```cpp
float ClampFloat(float v, float lo, float hi) {  // definition → becomes illegal C++
result = ClampFloat(x, 0, 1);                     // call site → correct
```
When a function **definition** matches, it becomes a malformed qualified definition in the wrong namespace. For C++ refactoring, always handle definitions and call sites in separate steps with exact string matching.

### 4. Replace strings must be syntactically valid

A typo like `s/DegToRad\(/ns::DegToRad</g` (the `<` should be `(`) produced `DegToRad<value)` across 5 files. One character destroyed 5 translation units. In batch mode, this is invisible until the next build. Use the Edit tool's unique-string guarantee to catch mismatches early.

### 5. Include insertion must match actual include structure

Different files have different include patterns (`"local.h"`, `<system>`, `"common/..."`). A single perl pattern for adding includes will randomly miss files whose include structure doesn't match. Check each file's actual includes before editing, or use the Edit tool with surrounding context to guarantee placement.

### 6. C++ language version constrains refactoring options

This project is C++11. Template variables (`template<typename T> constexpr T kPi = ...`) require C++14. This forced `kPi<float>()` function-call syntax instead of `kPi<float>` variable syntax, making every call site change syntactically invasive. When planning a cross-file rename, verify the target syntax is valid in C++11 before starting.

### Summary checklist before any batch edit across >3 files

- [ ] Can this be done in batches of 3-5 with `cmake --build` between each?
- [ ] Does the regex distinguish function definitions from call sites?
- [ ] Is the replacement string free of typo-level errors?
- [ ] Does every target file have the same include structure for include insertion?
- [ ] Is the new syntax valid in C++11? 

## Done Means
- The chosen preset builds successfully.
- Relevant tests pass for the chosen preset.
- New public API or significant logic changes include new or updated tests under `tests/`.
- Documentation stays accurate for changed behavior.

note: Your mileage may vary. Not all of these rules are necessarily optimal for every setup. Like anything else, feel free to break the rules once...
- you understand when & why it's okay to break them.
- you have a good reason to do so.
