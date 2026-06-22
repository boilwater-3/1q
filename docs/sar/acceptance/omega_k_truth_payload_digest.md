# Omega-K Truth Payload Digest Verifier Acceptance Report

## Scope

Stage 125 implements portable C++11 SHA-256 verification over exact truth
payload bytes.

## Accepted Behavior

- Requires a nonzero request identifier and a valid declared SHA-256 digest.
- Hashes exact payload bytes without parsing or normalization.
- Reports lowercase computed SHA-256 and matched or mismatched status.
- Accepts uppercase or lowercase declared hexadecimal digests.
- Rejects malformed requests atomically without publishing a computed digest.

## Verification

- Standard SHA-256 empty-string and `abc` vectors: passed through unit suite.
- Tampered payload mismatch and invalid-request coverage: passed.
- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed before acceptance recording.

## Boundary

A digest match establishes payload integrity relative to the manifest only. It
does not establish independent provenance or physical image correctness.
