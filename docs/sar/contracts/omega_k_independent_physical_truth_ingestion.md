# Omega-K Independent Physical Truth Ingestion Contract

## Purpose

This contract defines how an independently generated physical point-target
dataset may enter the Omega-K image acceptance process.

## Provenance

Every dataset shall include:

- a stable dataset identifier and schema version;
- the generator, measurement system, or external reference implementation;
- the generation or acquisition date;
- the responsible source and review record;
- a statement that the dataset does not reuse the executor under test;
- a cryptographic digest covering all truth data and metadata.

Datasets without a verifiable independence statement or digest shall be
rejected.

## Required Physical Metadata

The dataset shall explicitly provide:

- carrier frequency, bandwidth, propagation speed, and waveform definition;
- platform trajectory and coordinate reference frame;
- point-target position and reflectivity;
- sampling rates, axis ordering, units, and Fourier sign conventions;
- expected absolute slant range and azimuth coordinate;
- expected peak complex response and quality tolerances;
- common-support classification and any edge-case designation.

No required physical value may be inferred from filename, matrix dimensions, or
repository location.

## Ingestion Boundary

The ingestion layer shall:

- parse a versioned structured manifest;
- verify the declared digest before publishing data;
- validate finite values, dimensions, units, and monotonic axes;
- reject unknown required-field versions;
- preserve the original provenance record in the evaluation result;
- produce the acceptance evaluator request without modifying truth values.

The ingestion layer shall not run the imaging executor, derive expected values,
or relax tolerances.

## Test Fixtures

Synthetic repository fixtures may validate parser and evaluator behavior, but
shall be labeled non-physical and shall not authorize production image
acceptance.

## Acceptance Boundary

Production Omega-K physical image acceptance remains blocked until at least one
externally generated or measured dataset satisfies this contract and passes the
point-target acceptance evaluator.
