# Omega-K Physical Truth Evaluation Eligibility Gate Report

## Scope

Stage 129 implements a gate that determines whether successfully ingested truth
is eligible to enter physical point-target evaluation.

## Accepted Behavior

- Rejects unsuccessful ingestion results.
- Keeps non-physical and non-independent datasets ineligible.
- Requires source and acquisition-date provenance.
- Requires the computed and declared SHA-256 digests to match
  case-insensitively.
- Publishes only the eligible dataset identifier.
- Does not claim that an eligible dataset passes image-quality evaluation.

## Verification

- Windows default Debug build: passed.
- Windows Eigen 3.3.9 Debug build: passed.
- Windows default full CTest suite: 25/25 passed.
- Windows Eigen 3.3.9 SAR C++11 compatibility contract: passed.
- `git diff --check`: passed.
- Targeted runtime launch remains pending because managed sandbox spawn setup
  refresh failed.

## Boundary

Eligibility authorizes evaluation only. Production physical-image acceptance
still requires the eligible external or measured dataset to pass the
point-target evaluator.
