# STM32 Gripper Actuator Controller — Firmware

Closed-loop position-control firmware for an STM32F446RE-driven robotic
gripper actuator. Redesigned from time-based open-loop actuation to
encoder-feedback PD control for repeatable motion, matching the behaviour
described on the project page.

Bare HAL C, no CubeMX regeneration required to read it — the low-level init
for every peripheral lives in the driver modules, so it drops straight into a
standard STM32Cube/HAL project (see **Build & integration**).

---

## How the code maps to the feature list

| Feature | Where |
|---|---|
| PWM motor drive | `motor.c` — TIM1_CH1, 20 kHz, direction via IN1/IN2 |
| Quadrature encoder feedback | `encoder.c` — TIM2 32-bit, x4 counting, hardware counter |
| UART command interface (1-byte, interrupt) | `comms.c` — USART2 RX-complete IRQ |
| Interrupt-based position updates | `main.c` TIM6 tick → `controller_tick()` at 1 kHz |
| PD control | `controller.c` — `run_position_control()` |
| Startup boost | `controller.c` — kick while off-target and stalled |
| Deadzone compensation | `controller.c` — minimum effective duty |
| Velocity-aware arrival detection | `controller.c` — close **and** stopped, debounced |
| Software limits | `controller.c` — targets clamped to `[POS_MIN, POS_MAX]` |
| Homing-settle calibration | `controller.c` — `run_homing_seek/settle()` |

Everything you'd want to change (pins, gains, limits, homing behaviour) is in
**`Core/Inc/config.h`** — the rest of the code has no magic numbers.

---

## Wiring (Nucleo-F446RE + TB6612FNG-style driver)

| Signal | MCU pin | Notes |
|---|---|---|
| Motor PWM (PWMA) | PA8 | TIM1_CH1, AF1, 20 kHz |
| Direction AIN1 | PB4 | GPIO output |
| Direction AIN2 | PB5 | GPIO output |
| Driver enable STBY | PB10 | held high |
| Encoder A | PA0 | TIM2_CH1, AF1 |
| Encoder B | PA1 | TIM2_CH2, AF1 |
| UART TX | PA2 | USART2 → ST-Link virtual COM |
| UART RX | PA3 | USART2 |

Works unchanged with DRV8833 / L298N-style drivers (one PWM line + two
direction lines). If you use two-PWM drivers instead, drive one channel and
tie the other logic line — or ask and I'll adapt `motor.c`.

---

## Serial command interface

115200 8N1 on the ST-Link virtual COM port. Every command is a single byte,
handled in the RX interrupt.

```
h        home: seek stop, settle, zero the encoder, back off
o / c    open / close
[ / ]    go to min / max limit
+ / -    nudge open / close (NUDGE_STEP counts)
0-9      go to 0%..90% of travel
s        stop and hold current position
x        disable (coast)      e   re-enable (hold here)
p / ?    print status
```

Status line (all integers, so no float-printf linker flag needed):

```
STATE=MOVING  TGT= 2100 POS= 1873 ERR=  227 VEL=  412 c/s PWM=  268
```

---

## Control design

Run at **1 kHz** from the TIM6 period-elapsed interrupt. Each tick:

1. **Sample & estimate velocity.** Read the hardware encoder counter, take the
   per-tick delta, and smooth it with a light EMA (`VEL_FILTER_ALPHA`).
2. **Safety limit check.** If position runs past a hard limit, coast.
3. **PD law:** `u = Kp·error − Kd·velocity`. Derivative acts on measured
   velocity (not error difference), which is cleaner with encoder noise.
4. **Startup boost.** When off-target but not yet moving, override with a fixed
   kick (`BOOST_PWM`) for up to `BOOST_TICKS` to break static friction and take
   up backlash. Re-armed on every new target.
5. **Deadzone compensation.** If the PD output is non-zero but below the duty
   that actually moves the motor (`MIN_MOVE_PWM`), bump it up to that floor —
   only while still off-target, so it doesn't chatter at the setpoint.
6. **Velocity-aware arrival.** Declared "arrived" only when `|error| ≤ POS_TOL`
   **and** `|velocity| ≤ VEL_TOL`, held for `ARRIVE_DEBOUNCE` ticks. This
   rejects the moment of overshoot where position looks right but the actuator
   is still coasting. A `POS_HYST` band prevents re-triggering on tiny drift.

### Homing-settle calibration (`h`)

`SEEK` drives gently toward the closed mechanical stop at `HOME_PWM`. Once real
motion has been seen and velocity collapses for `HOME_STALL_DEBOUNCE` ticks,
the stop is confirmed (an initial grace window stops the standstill at t=0 from
being read as the stop). `SETTLE` rests on the stop for `HOME_SETTLE_TICKS` so
the mechanics relax, then zeroes the encoder there and backs off by
`HOME_BACKOFF`. A `HOME_TIMEOUT_TICKS` guard aborts if no stop is ever found.

---

## First-run calibration

1. **Encoder/motor sign.** Flash, open a terminal, send `h`. If homing drives
   *away* from the stop, or `o` makes `POS` shrink instead of grow, set
   `MOTOR_INVERT 1` in `config.h`.
2. **Travel range.** After homing, jog with `+`/`-` to the open stop, read
   `POS`, and set `POS_MAX` (and `OPEN_POS`/`CLOSE_POS`) to match your actuator.
3. **Deadzone.** From rest send small nudges; raise/lower `MIN_MOVE_PWM` until
   the smallest command reliably produces motion without slamming.
4. **Gains.** Raise `KP` until it converges briskly, then raise `KD` until
   overshoot/oscillation is damped. Tighten `POS_TOL`/`VEL_TOL` last.

Starting defaults assume ~2200 counts of travel and target error in the
±15–40 count range, consistent with the measured closed-loop results.

---

## Build & integration

This is application code, not a full toolchain project — it needs the STM32
HAL, a linker script, and the F446 startup file, exactly as any HAL project
does. Two easy paths:

**A. Into a CubeMX project (fastest):**
1. New STM32CubeIDE project for **STM32F446RETx**, HAL, no peripherals needed.
2. Copy the six `.c` files into `Core/Src/` and the six `.h` files into
   `Core/Inc/`, replacing the generated `main.c`/`main.h`.
3. In `stm32f4xx_it.c`, **remove** any generated `SysTick_Handler`,
   `TIM6_DAC_IRQHandler`, `USART2_IRQHandler` (this firmware defines them in
   `main.c` — keep exactly one copy of each), or vice-versa.
4. Build & flash. Open the virtual COM port at 115200.

**B. Bare Makefile/CMake HAL project:** add these sources alongside the CMSIS +
HAL sources, `system_stm32f4xx.c`, the F446 startup `.s`, and the flash linker
script; ensure `stm32f4xx_hal_conf.h` enables the `TIM`, `UART`, `GPIO`, `RCC`,
`PWR` modules.

Clock assumptions live in `config.h` (`*_TIM_CLK_HZ`). If you don't run at
180 MHz, update those two values and the derived PWM/tick dividers follow
automatically.

---

## Files

```
Core/
├── Inc/
│   ├── config.h        all tunables: pins, timers, gains, limits, homing
│   ├── main.h          shared HAL handle externs + prototypes
│   ├── motor.h         .c   TIM1 PWM + direction driver
│   ├── encoder.h       .c   TIM2 quadrature counter
│   ├── controller.h    .c   PD loop, boost, deadzone, arrival, homing, limits
│   └── comms.h         .c   USART2 1-byte command interface + telemetry
└── Src/
    ├── main.c          clock, init, TIM6 tick, NVIC, IRQ handlers
    ├── motor.c
    ├── encoder.c
    ├── controller.c
    └── comms.c
```
