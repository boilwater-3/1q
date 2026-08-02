# AR and SBIRS Repair Patterns

Read only the sections relevant to the active module. Re-check live code because identifiers and test names can drift.

## Pattern map

| Concern | AR lesson | SBIRS lesson | Reusable rule |
|---|---|---|---|
| Temporal semantics | Apply accepted external LPI/ECCM decisions on the next successful cycle | Apply runtime mode/config changes immediately, then observe them on the next executed cycle | Name the exact cycle boundary and define non-executed cycles |
| Choice model | External proposals replace the internal baseline for the target cycle | Three mutually exclusive tracking modes replace interacting booleans | Encode mutually exclusive states explicitly; define replace versus merge |
| Provenance | Match source cycle and batch before accepting an external response | Record stable tracking source in attribution/debug/replay | Persist the provenance needed to explain every applied choice |
| State ownership | Let Session own pipeline restore and Controller own decision state | Let Pipeline own scheduler/pointing/tracking migration | Assign one owner per cumulative state; never duplicate snapshots |
| Physical proof | Assert detection-margin effects from LPI/ECCM controls | Assert common physical gates while noisy output remains non-feedback | Test observable physics/state consequences, not only DTO values |
| Replay | Persist accepted pending decisions and next-cycle replacement | Persist mode/backend/source and random consumption | Replay every semantic input that can change a later output |
| Documentation | Promote settled decision semantics into AR `design.md` | Delete settled TruthAssisted draft after promoting the contract | Maintain one authority and retire historical drafts |

## AR: external decision and next-cycle control

Apply this pattern to asynchronous or in-process external control:

1. Preserve the internal decision engine as the baseline and fallback.
2. Expose stable observation/response DTOs; keep algorithm state and internal tactical types private.
3. Match the response to both source cycle and source batch, or equivalent provenance keys.
4. Define accepted response semantics explicitly:
   - proposals replace rather than silently merge the internal baseline;
   - an accepted empty proposal set explicitly disables the controlled feature for the target cycle;
   - a missing or invalid response leaves the internal fallback intact.
5. Apply the pending decision on the next **successful** cycle. Preserve it across validation rejection, standby, or powered-off cycles if those cycles do not consume it.
6. Snapshot the pending decision with its owner. Restore pipeline/environment state only through the session-level owner; restore controller decision state through the controller.
7. Prove the resulting signal or detection effect. Do not accept a test that only observes the selected profile.
8. Record the observation, accepted response, provenance, applied source, and final control in trace/replay when replay is in scope.

Use a mailbox analogy for explanation: a valid response is a sealed instruction for the next train that actually departs; a cancelled departure must not consume the instruction.

## SBIRS: tracking modes, physical gates, and random streams

Apply this pattern to oracle, truth-assisted, sensor-like, or estimated modes:

1. Separate simulation mode from estimator backend. Avoid booleans that permit contradictory combinations.
2. Give each incompatible live state a distinct internal state and stable external attribution source.
3. Keep raw sensor output free of simulation-only target identity, truth source, and internal state enums.
4. Let truth-assisted modes use truth only for the explicitly approved command/output role. Keep actuator dynamics, FOV geometry, SNR, coasting, and loss gates active.
5. Generate sensor-like noise only after the physical observation succeeds. Never feed display/output noise back into pointing or gate decisions unless the contract explicitly models a measurement-driven loop.
6. Split random streams by semantic consumer. Use fixed-width integer state, fixed domain tags, explicit no-sample cycles, per-stream snapshot state, and seed-local reset.
7. Define runtime migration by compatibility:
   - retag compatible truth modes in place and preserve lock/actuator/random state;
   - release incompatible truth↔estimated state for deterministic reacquisition;
   - release only active estimated tracks for estimator backend changes;
   - treat equal-value patches as valid no-ops.
8. Compare modes with paired multi-cycle tests that assert identical gates, coasting, failure counters, loss cycles, and resource release while allowing only the intended output difference.
9. Register unresolved fidelity boundaries instead of pretending the model is real hardware. Typical questions include truth-derived diagnostic range, stage-specific error statistics, target-order-dependent random allocation, and truth-seeded estimator initialization.

Use an instrument analogy for explanation: Strict reports the ruler's true reading; Sensor-like adds instrument error after the same pass/fail inspection; the noisy display must not move the inspected object.

## Shared runtime-state migration rules

Build an impact matrix before coding:

| Change class | Preserve | Reset/retag/release |
|---|---|---|
| Cosmetic or ordinary threshold/config value | compatible cumulative state | only directly invalid counters |
| Statistical R/Q or gate-policy parameter | state mean/covariance where valid | dependent consecutive counters |
| Seed | unrelated streams and bindings | only owned stream/epoch |
| Compatible mode | locks, actuators, queues, random state | retag state |
| Incompatible algorithm family | unrelated targets and global phase | release incompatible target state |
| Capacity shrink | deterministic low-index survivors | out-of-range bindings and owned state |
| Inactive mode | explicitly frozen phase/random state | targets, cues, locks, pointing, tracking as contracted |

Validate all cross-owned mappings before mutating live state. Use temporary scheduler/coordinator/filter candidates when an operation can fail.

## Failure patterns to avoid

- Trusting review prose without tracing the live consumer.
- Calling a declared config knob implemented when no execution path reads it.
- Adding replay after implementation and discovering missing timing state later.
- Duplicating pipeline state inside a controller snapshot.
- Treating debug or attribution data as raw sensor output.
- Claiming truth-assisted output reproduces real hardware.
- Reusing one random stream without defining consumption and target ordering.
- Using a system FlatBuffers generator instead of the repository-pinned toolchain.
- Guessing CMake target names instead of reading the registry.
- Keeping a settled review draft beside the authoritative `design.md`.
