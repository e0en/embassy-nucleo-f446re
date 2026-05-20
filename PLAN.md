# Output Shaft Angle Estimator Integration Plan

## Assumptions

- `OUTPUT_SHAFT_REDUCTION_RATIO` keeps its current signed meaning.
- `rotor_phase` is the wrapped primary-encoder phase used for commutation.
- `output_phase` is the wrapped output-shaft phase resolved from the dual encoders.
- `output_velocity` is derived as `rotor_velocity / OUTPUT_SHAFT_REDUCTION_RATIO`.
- Before zero alignment is valid, fused `SensorReading` is not considered available.

## Implementation Order

## Execution Rules

- When a step is completed, make a dedicated commit for that step before moving on.
- Mark completed steps directly in this file with a `Status: Completed` line under the step heading.

### 1. Rename and constrain the output resolver

Status: Completed

- Update `foc/src/output_angle.rs`.
- Rename `estimate_output_angle()` to `resolve_output_phase()`.
- Make the contract explicit: zeroed wrapped encoder phases in, wrapped output phase in `[0, TAU)` out.
- Keep signed reduction ratio semantics.

Verification:

- Unit tests cover ideal reconstruction, wrap boundaries, noise tolerance, and signed ratio `-19`.

### 2. Introduce raw and fused sensor types

Status: Completed

- Update `foc/src/angle_input.rs` and exports in `foc/src/lib.rs`.
- Separate unaligned raw sensor data from aligned fused sensor data.
- Recommended model:
  - `RawSensorReading { primary_phase, secondary_phase, rotor_velocity, dt }`
  - `SensorReading { output_phase, rotor_phase, rotor_velocity, dt }`

Verification:

- Type usage compiles cleanly and prevents semantic mixing at call sites.

### 3. Make the primary tracker rotor-specific

- Update `stm32-kit/src/encoder_correction.rs`.
- Keep the existing primary-encoder tracking logic, but make its role explicit:
  - wrapped `rotor_phase`
  - continuous `rotor_velocity`
  - `dt`
- Do not let this tracker own output-side phase reconstruction.

Verification:

- Existing velocity-stability behavior remains covered.
- Wrap-crossing behavior remains stable.

### 4. Add explicit zero-alignment validity

- Update `stm32-kit/src/main.rs`.
- Add a validity flag such as `ZERO_ALIGNMENT_VALID`.
- Set it on config load and `SetZeroPosition`.
- Clear it on recalibration / config clear paths.

Verification:

- Fused sensor computation is blocked while alignment is invalid.
- Validity is independent from the numeric value of the offsets.

### 5. Rework snapshot storage around raw sensor meaning

- Update snapshot structures in `stm32-kit/src/main.rs`.
- Replace ambiguous fields with explicit sensor-side names.
- Recommended fields:
  - `output_phase`
  - `rotor_phase`
  - `rotor_velocity`
  - `dt`
  - `aligned`

Verification:

- Snapshot read/write helpers remain coherent with the double-buffer scheme.

### 6. Refactor `encoder_task()` into a raw sensor producer

- Update `stm32-kit/src/main.rs`.
- Primary path:
  - read raw angle
  - apply correction
  - apply zeroing as needed
  - update rotor tracker
- Secondary path:
  - read raw angle
  - apply zeroing as needed
- Stop at producing `RawSensorReading` in this step.

Verification:

- Primary-only and secondary-present paths both behave consistently.
- Zero-offset version changes still reset the relevant tracking state.

### 7. Add raw-to-fused sensor resolution

- Add a dedicated module, preferably `stm32-kit/src/gear_sensor.rs`.
- Convert `RawSensorReading` plus alignment validity into `Option<SensorReading>`.
- Call `resolve_output_phase()` only when aligned.

Verification:

- Unaligned input yields no fused reading.
- Aligned input yields expected `output_phase`.
- Signed reduction ratio direction is preserved.

### 8. Add an output-angle tracker

- Add `stm32-kit/src/output_tracker.rs`.
- Track continuous `output_angle` from wrapped `output_phase`.
- Do not estimate velocity here.

Verification:

- Output angle stays continuous across `0 / 2pi`.
- Multi-turn continuity and reset behavior are covered by tests.

### 9. Split controller input from sensor input

- Update `foc/src/controller.rs`, `foc/src/angle_input.rs`, and `stm32-kit/src/foc_isr.rs`.
- Introduce a controller-facing input type such as:
  - `ControlReading { output_angle, output_velocity, rotor_phase, dt }`
- Compute:
  - `output_velocity = rotor_velocity / OUTPUT_SHAFT_REDUCTION_RATIO`
- Keep electrical-angle calculation based on `rotor_phase`.
- Keep controller state angle/velocity in output-shaft coordinates.

Verification:

- Controller tests confirm commutation still follows `rotor_phase`.
- Controller state reflects output-side angle and velocity.

### 10. Convert the runtime snapshot to final control input

- Update `stm32-kit/src/main.rs` and `stm32-kit/src/foc_isr.rs`.
- Make `encoder_task()` publish controller-ready readings.
- Keep ISR responsibility minimal: read snapshot and pass it through.

Verification:

- Snapshot reader/writer semantics remain correct.
- Alignment transitions do not break ISR consumption.

### 11. Switch firmware command/feedback semantics to output coordinates

- Update `stm32-kit/src/main.rs` and `stm32-kit/src/foc_isr.rs`.
- Make `AngleRef`, `SpeedRef`, `Angle`, and `Speed` consistently output-side.
- Remove or shrink the old input/output conversion helpers.
- Gate angle/velocity/impedance commands while unaligned.

Verification:

- Unaligned state blocks angle-based and velocity-based control paths.
- Aligned state reports and consumes output-side values consistently.

### 12. Update the monitor in lockstep with firmware semantics

- Update `monitor/src/main.rs`.
- Update `can_message` only if protocol structure must change.
- Make the monitor treat angle and speed as output-side values.
- Surface alignment state clearly enough to avoid misleading controls.
- Keep the zeroing workflow text aligned with the new model.

Verification:

- `can_message` round-trip tests still pass.
- `monitor` builds cleanly against the updated firmware-facing meaning.

### 13. Finish zeroing and flash persistence integration

- Update `stm32-kit/src/main.rs` and `stm32-kit/src/flash_config.rs`.
- On `SetZeroPosition`:
  - store both offsets
  - mark alignment valid
  - reset rotor/output trackers
  - reset controller state
- Ensure save/load preserves the aligned startup path.

Verification:

- Save/load round-trip preserves alignment behavior.
- Zeroing immediately reinitializes control state cleanly.

### 14. Final verification pass

- Run `cargo fmt`.
- Run `cargo test -p foc`.
- Run `cargo test -p can_message`.
- Run `cargo build -p monitor`.
- Run the relevant firmware-side verification command set that remains available in this workspace.

Verification:

- Formatting and all step-owned tests pass together.
