# Project Overview

The project is a simulation model library that provides external services which including model such as airborne-radar and electronic countermeasures set/equipment.

Tech stack:
- C++ standard : C++11.
- Build system : CMake 
- Package manager : Conan
- Logging : spdlog
- Testing : GoogleTest
- Testing : GoogleMock
- Documentation : Doxygen
- Event bus : eventpp
- Math library : Eigen
- Math library : Sophus

# Repository Structure

This project strictly follows the principle of public/private header separation (Pitchfork Layout Standard). The architecture system maps to the entire workspace as follows:

```text
1q/
├── include/1q/                  # [Public API Root Directory]
│   ├── api.hpp                  # Global export macros and basic definitions
│   └── airborne_radar/          # radar signal processing simulation api
│   └── ej/                      # electronic jamming simulation api
├── src                          # [Private implementation and internal concrete classes]
│   ├── airborne_radar/          # Radar module. Has a sub GEMINI.md in this dirctory.
│   ├── ej/                      # electronic jamming (EJ) module. Has a sub GEMINI.md in this dirctory.
├── doc/                         # Architecture blueprints, design documents, and image resources
├── tests/                       # Unit and integration tests (GTest)
├── examples/                    # External call demonstration examples
├── cmake/                       # CMake modules and toolchain configurations
├── tools/                       # Helper development scripts and maintenance tools
├── CMakeLists.txt               # Comprehensive build script
├── conanfile.txt / vcpkg.json   # Dependency manager configuration files
└── CMakePresets.json            # Cross-platform build presets
```

# Architecture Principles

- No C++ Exceptions.
- Log Critical Paths and Events.
- Inter-module blocking calls are forbidden.

# Coding Standards

- Use Google C++ Style Guide. 
- Favor Composition over Inheritance.
- Prefer Forward Declarations.
- Namespace-Directory Mapping.
- All functions must be documented with Doxygen comments written in Chinese.
- Each file must include a header description in Chinese.
- Centralize module assembly in entry points (e.g., a `Builder` or `main()`), where concrete components are instantiated and injected.
- High-level policy layers must not depend on concrete infrastructure implementations. Dependencies must be injected via abstract interfaces.

# Build & Test

- Test dependencies must not pollute the top-level build; they should be managed in the `[test_requires]` section of `conanfile.py` (or `vcpkg.json`).
- To test private implementations (in `src/`), explicitly expose the `src` directory to the test target in `tests/CMakeLists.txt`.
- Configuration: `cmake --preset llvm-ninja-debug-local`
- Build: `cmake --build --preset llvm-ninja-debug-local`
- Test: `ctest --preset llvm-ninja-debug-local`

# AI Behavior

- Provide concise diffs.
- Do not assume missing requirements.
- Use Chinese when thinking and communicating with users.

# Terminal Usage

- Prefer background execution and retrieve complete output afterward when terminal output may be truncated or hidden.
- For commands with large outputs, redirect to file using `> output.txt 2>&1` and read selectively.
- Never read entire files unless necessary.
- Always search first using rg or grep.
- Use line numbers to locate relevant sections.
- Only read small ranges using sed/head/tail.
- Avoid commands that produce large outputs.





