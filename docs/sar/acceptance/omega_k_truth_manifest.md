# Omega-K Versioned Truth Manifest Parser Acceptance Report

## Scope

Stage 123 implements strict parsing and structural validation for a versioned
Omega-K truth manifest using an explicitly non-physical synthetic fixture.

## Accepted Behavior

- Requires the exact versioned manifest header and schema version.
- Parses fields in a fixed documented order with strict types.
- Validates finite physical values and tolerances.
- Validates the declared SHA-256 digest field as exactly 64 hexadecimal
  characters.
- Rejects unknown versions, reordered fields, malformed values, and trailing
  content atomically.
- Preserves the fixture's non-physical and non-independent labels.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed.

## Boundary

The parser validates digest syntax but does not verify the digest against a
payload. Cryptographic verification and production physical evidence remain
deferred.
