# Omega-K Eligible Truth Evaluation Orchestration Decision

## Decision

Approve an orchestration layer that invokes point-target image evaluation only
after physical truth eligibility succeeds.

## Binding Requirements

The orchestrator shall require:

- a successful eligibility result;
- the successful ingestion result used to produce that eligibility result;
- an exact match between eligible dataset identifier and ingested manifest
  dataset identifier;
- explicit image coordinate axes and numerical image candidate.

The orchestrator shall construct the evaluator request from the ingested
manifest truth and tolerances without modification.

## Outcome Separation

The result shall distinguish:

- orchestration rejection;
- eligible evaluation that fails quality metrics;
- eligible evaluation that passes quality metrics.

Eligibility shall never be reported as a quality pass.

## Boundary

Synthetic fixtures may validate orchestration rejection and plumbing. Only an
eligible external or measured dataset may produce a physical quality decision.
