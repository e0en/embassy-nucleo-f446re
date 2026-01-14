# STM32 BLDC Motor Controller

Rust FOC (Field-Oriented Control) motor controller for STM32G431RB.

## Structure

- `stm32-kit/` - Firmware (Embassy async, runs on MCU)
- `foc/` - FOC algorithms library (no_std)
- `can_message/` - CAN protocol definitions (no_std)
- `monitor/` - Desktop GUI for monitoring (egui)

## Build

```bash
# Firmware
cd stm32-kit && cargo build --release

# Monitor
cd monitor && cargo build --release
```

Target: `thumbv7em-none-eabihf` (Cortex-M4F). Requires nightly Rust.

## Key Files

- `stm32-kit/src/main.rs` - Init, task spawning, CAN command handling
- `stm32-kit/src/foc_isr.rs` - FOC loop (runs in ADC interrupt)
- `stm32-kit/src/cordic.rs` - Hardware CORDIC for sincos/atan2
- `stm32-kit/src/drv8316.rs` - Motor driver (DRV8316) control
- `stm32-kit/src/gm3506.rs` - Motor profile (pole pairs, PID gains)
- `foc/src/controller.rs` - Clarke/Park transforms, control modes

## Hardware

- MCU: STM32G431RB
- Driver: DRV8316 (3-phase gate driver with CSA)
- Encoder: AS5048A (14-bit absolute, SPI)
- Motor: GM3506 (11 pole pairs)

## Patterns

- FOC loop in ADC ISR for deterministic timing
- Hardware CORDIC instead of libm for trig
- Embassy framework for async tasks
- defmt + RTT for debug logging

## Code Style

- Prefer shorter functions.
- Utilize rust's zero-cost abstraction to shorten functions
- `f32` only, no f64 (CORDIC hardware, embedded constraints)
- `p_` prefix for peripheral variables (e.g., `p_adc`, `p_spi`)
- `f_` prefix for function pointers (e.g., `f_sincos`)
- `defmt::Format` derive on error/status types for RTT logging
- `#[inline(always)]` on ISR functions (`foc_isr.rs`)
- PAC register access in ISR (bypass Embassy for speed)
- `AtomicU32` + `to_le_bytes`/`from_le_bytes` for ISR↔task f32 sharing
- `CriticalSectionRawMutex` for channels accessed from ISR
- `ThreadModeRawMutex` for task-only channels
- Motor configs as module-level `const` (see `gm3506.rs`)
- `#[allow(dead_code)]` on alternative motor profiles

## Comments

- Avoid comments unless describing intent not expressed by code
- Note execution context: "called from thread mode", "accessed from ADC interrupt"
- Note performance on hot path: "keep it fast!", "no Embassy overhead"
- Document hardware constraints inline (CORDIC input ranges, voltage divider ratios, pin mappings)

## Code Validation

- Every commited code must pass all tests, and formatted properly with `cargo fmt`
