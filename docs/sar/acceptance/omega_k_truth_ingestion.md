# Omega-K Atomic Truth Ingestion Gate Acceptance Report

## Scope

Stage 127 combines strict manifest parsing and exact-byte SHA-256 verification
into one atomic truth-ingestion gate.

## Accepted Behavior

- Requires a nonzero ingestion request identifier.
- Reports manifest rejection separately from digest rejection and mismatch.
- Publishes the parsed manifest, original payload bytes, and computed digest
  only after a successful parse and digest match.
- Preserves physical-evidence and independence classifications exactly.
- Rejects mismatched or malformed input atomically.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed.

## Boundary

The synthetic integrity fixture validates ingestion behavior but remains
non-physical and non-independent. Production image acceptance still requires
an external or measured dataset.
