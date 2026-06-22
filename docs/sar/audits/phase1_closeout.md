# SAR Phase 1 Closeout Report

Date: 2026-06-12

## Delivery boundary

The current `1.1.4.4 SAR` work is closed at Stage 139. The repository contains
the bounded Omega-K validation path and the bounded PGA support, gradient
estimation, and truth-comparison path.

Stage 140 and later PGA work is deliberately deferred. Phase-gradient
integration, unwrap, iterative autofocus, and production image correction are
not authorized by the completed evidence.

## Buildable deliverables

- The normal Windows preset remains the full-project build and test path.
- Visual Studio 2015 can compile the isolated `sar_core` target.
- Visual Studio 2015 can compile and run `sar_legacy_toolchain_smoke`, which
  executes the completed PGA truth-to-estimator-to-comparison chain.

The complete repository cannot be built with Visual Studio 2015 because the
currently imported JSBSim sources use C++17 library facilities such as
`std::is_enum_v`, and the vendored GoogleTest version also requires a newer
compiler. Those third-party requirements are outside the SAR closeout scope.

## Remaining evidence and deferred work

- Omega-K physical acceptance still requires an eligible external or measured
  point-target truth package.
- PGA integration, unwrap, stopping criteria, iterative correction, and
  production image modification remain deferred.
- A full-repository VS2015 port would require selecting or maintaining legacy
  compatible JSBSim and GoogleTest dependencies.

## Acceptance commands

```powershell
cmake --build --preset windows-vs2026-debug --parallel 8
ctest --preset windows-vs2026-debug --output-on-failure -j 8

cmake -S . -B build/windows-vs2015-debug-local -G "Visual Studio 14 2015" -A x64 -DENABLE_TESTING=OFF -DONEQ_JSBSIM_FROM_SOURCE=OFF
cmake --build build/windows-vs2015-debug-local --config Debug --target sar_core
cmake --build build/windows-vs2015-debug-local --config Debug --target sar_legacy_toolchain_smoke
build/windows-vs2015-debug-local/Debug/bin/sar_legacy_toolchain_smoke.exe
```

VS2015 source-tree CRLF/BOM normalization is disabled by default so configuring
does not dirty the checkout. `ONEQ_VS2015_NORMALIZE_SOURCE=ON` remains an
explicit fallback for legacy parser issues outside the isolated SAR targets.

## Final verification

- Modern Windows debug build: passed.
- Modern full CTest: 25/25 passed.
- VS2015 `sar_core` rebuild: passed.
- VS2015 `sar_legacy_toolchain_smoke` rebuild and execution: passed, exit 0.
- `git diff --check`: passed.
