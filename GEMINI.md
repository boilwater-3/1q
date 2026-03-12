# Project Overview

The project is a simulation model library that provides external services which including model such as airborne-radar and electronic countermeasures set/equipment.

Tech stack:
- C++ standard : C++11.
- Build system : CMake 
- Package manager : Conan
- Build tool : Ninja
- Logging : spdlog
- Testing : GoogleTest
- Testing : GoogleMock
- Documentation : Doxygen
- Diagram (PlantUML) : plantuml 
- Diagram (Mermaid) : mermaid-cli 
- Event bus : eventpp
- Math library : Eigen
- Math library : Sophus

# Repository Structure

This project strictly follows the principle of public/private header separation (Pitchfork Layout Standard). The architecture system maps to the entire workspace as follows:

```text
1q/
├── include/1q/                  # [Public API Root Directory]
│   ├── api.hpp                  # Global export macros and basic definitions
│   └── airborne_radar/          # Radar core library abstract interfaces
│   └── ej/                      # electronic jamming (EJ) core library abstract interfaces
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
- Decouple Control Plane from Data Plane: pipeline nodes should only pass state context, and actions must be encapsulated as commands for deferred, unified execution.
- Decouple high-frequency, cross-layer data using an internal event bus (publish/subscribe) to avoid point-to-point coupling.
- Strictly prohibit SQL/disk I/O in hot paths; static data must be loaded into memory during the initialization phase.

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

- Test dependencies must not pollute the top-level build; they should be managed in the `[test_requires]` section of `conanfile.txt`.
- Do not use globbing for source files; new `.cpp` files must be explicitly added to the module's source list.
- To test private implementations (in `src/`), explicitly expose the `src` directory to the test target in `tests/CMakeLists.txt`.

# AI Behavior

- Prior to modifying files in any subdirectory, proactively search for a local GEMINI.md file. Local instructions take precedence over global ones for that specific scope.
- Provide concise diffs
- Explain non-trivial decisions
- Do not assume missing requirements
- Leverage CLI tools for maximum efficiency
- Use Chinese when communicating with users
- When running terminal commands, prefer background execution and retrieve output via get_terminal_output





