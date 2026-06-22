# Omega-K Atomic Truth Ingestion Gate Decision

## Decision

Approve an atomic truth-ingestion gate that combines strict manifest parsing
with exact-byte payload digest verification.

The gate may publish a parsed manifest and payload only when both operations
succeed and the computed digest matches the manifest declaration.

## Classification

The gate shall preserve the manifest's `physical_evidence` and
`independently_generated` classifications exactly.

A synthetic or non-independent fixture may pass parsing and integrity checks,
but shall remain ineligible for production physical image acceptance.

## Required Behavior

The gate shall:

- accept the original manifest text and exact payload bytes;
- parse the manifest without changing it;
- verify the payload against the parsed digest;
- publish the manifest and payload atomically only on a digest match;
- report parsing, digest-request, and digest-mismatch failures separately;
- preserve all provenance and classification fields.

## Deferred Work

Production acceptance remains blocked until a physical, independently generated
dataset passes this gate and the point-target image acceptance evaluator.
