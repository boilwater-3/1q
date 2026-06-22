# SAR Phase Reference Contract

Date: 2026-06-22

## Purpose

This contract separates SAR phase-reference behavior from image-quality
measurement. The approved work extracts the existing RDA broadside reference
and global constant phase comparison into explicit internal helpers without
changing focusing algorithm selection.

## Approved Boundary

The phase-reference module may provide:

- native no-op mode;
- broadside center phase reference for L1 RDA range-compressed pulse history;
- global constant phase-offset estimation between equal-shaped complex images;
- application of a supplied global phase rotation.

The module shall use the existing SAR internal style: POD request/config
structures, free functions, `bool` success return, and output parameters.

## Broadside Center Reference

The broadside reference shall preserve the current RDA behavior:

- slow time is centered at the middle aperture row;
- cross-range position is `platform_velocity_mps * slow_time_s`;
- range bin distance is `max(col * range_bin_spacing_m, 0.5 * range_bin_spacing_m)`;
- slant range is `sqrt(range_m^2 + x_m^2)`;
- phase rotation is `exp(j * 4*pi*slant_range / wavelength)`.

The operation shall not alter sample magnitudes.

## Global Constant Phase

Global phase estimation shall use the existing full-image coherent comparison
contract:

- inputs must be nonempty equal-shaped complex matrices;
- zero-energy images are rejected atomically;
- only one constant phase offset may be removed;
- amplitude scale may be normalized for shape comparison.

Spatially varying phase errors, phase ramps, defocus, RCMC errors, or motion
residuals shall remain visible in RMS error and coherent correlation.

## Explicit Exclusions

- No Auto focusing selector enablement.
- No public configuration widening by default; public output may expose scalar
  phase-reference summary fields for replay and trace consistency.
- No phase unwrap, autofocus correction, or spatially varying phase repair.
- No relabeling of global phase comparison as radiometric or geometric
  acceptance.

## Acceptance

Acceptance requires:

- unit coverage for no-op, valid broadside reference, invalid inputs, and
  magnitude preservation;
- RDA output equivalence against the previous embedded implementation;
- global constant phase comparison coverage showing constant phase removal and
  spatially varying phase error preservation;
- focused SAR tests continuing to pass without threshold weakening.
