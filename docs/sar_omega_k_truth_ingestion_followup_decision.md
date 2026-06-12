# Omega-K Truth Ingestion Follow-up Decision

## Decision

The absence of an externally generated physical truth dataset blocks production
physical-image acceptance, but it does not block repository-side robustness
work.

Repository work may continue on deterministic validation, resource limits,
classification enforcement, and end-to-end synthetic pipeline checks, provided
that every synthetic result remains explicitly ineligible for physical-image
acceptance.

## Approved Next Boundary

Approve a truth-ingestion eligibility evaluator that determines whether an
ingested dataset is eligible to enter the physical point-target acceptance
path.

Eligibility requires:

- successful atomic ingestion;
- `physical_evidence=true`;
- `independently_generated=true`;
- a non-empty provenance source and acquisition date;
- a verified payload digest.

The evaluator shall not claim that an eligible dataset passes image-quality
acceptance. It only authorizes evaluation.

## Deferred Boundary

Final Omega-K physical-image acceptance remains blocked until an external or
measured eligible dataset is supplied and passes the point-target evaluator.
