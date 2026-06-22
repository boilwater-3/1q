# Omega-K Reference Phase and Absolute Range Mapping Contract

## Purpose

This contract defines the physical metadata and truth evidence required to
promote an Omega-K relative-delay intermediate into an absolute-range,
image-domain result. It does not authorize production integration by itself.

## Required Inputs

An implementation request shall explicitly provide:

- a unique request identifier;
- carrier frequency in hertz;
- propagation speed in metres per second;
- reference slant range in metres;
- the sign convention used by the range-frequency Fourier transform;
- the sign and analytic expression of the reference phase compensation;
- the uniformly spaced reduced range-frequency axis;
- the relative-delay-domain complex matrix;
- the azimuth-frequency or azimuth-time output coordinate definition;
- all transform normalization factors.

No input may be inferred from matrix dimensions, global state, or an implicit
FFT convention.

## Absolute Range Mapping

For two-way monostatic propagation, each relative delay sample shall map to
absolute slant range using:

`R_absolute = R_reference + c * tau_relative / 2`

where `R_reference`, `c`, and the relative-delay sign convention are explicit
request fields. The executor shall return both the relative-delay axis and the
absolute slant-range axis.

The implementation shall reject:

- non-finite or non-positive propagation speed;
- non-finite or negative reference slant range;
- a relative-delay axis inconsistent with the reduced frequency grid;
- an ambiguous or unspecified delay sign convention;
- any absolute range that is non-finite or negative.

## Reference Phase

The reference phase compensation shall be represented by an explicit analytic
expression and sign convention. The request shall identify the domain in which
the compensation is applied.

The implementation shall not silently choose conjugation, FFT direction, phase
sign, or carrier removal. A zero-phase or identity compensation is permitted
only when the request declares it and the independent truth case expects it.

## Output Coordinates And Normalization

The result shall report:

- absolute slant-range coordinates in metres;
- the azimuth coordinate and its physical unit;
- matrix shape and axis ordering;
- every FFT normalization factor applied;
- common-support reduction metadata needed to interpret the valid image area.

The result remains invalid as a physical image if any coordinate or
normalization convention is absent.

## Independent Point-Target Truth

Acceptance requires at least one independently generated point-target case
whose expected image location and complex response are not computed by the
executor under test.

The truth case shall verify:

- absolute range-bin location;
- azimuth-bin location;
- peak phase within a declared tolerance;
- peak magnitude and transform normalization;
- range and azimuth sidelobe behavior within declared tolerances;
- deterministic rejection when the target lies outside common support.

An FFT round trip or self-generated expected matrix is insufficient evidence.

## Atomic Behavior

Validation and all coordinate calculations shall complete before publishing a
result. Rejected requests shall not return a partial image, partial coordinate
axis, or partially compensated matrix.

## Acceptance Boundary

This contract freezes the next implementation boundary. Production Omega-K
image integration remains deferred until an executor and independent
point-target tests satisfy every requirement above.
