RCWS Gimbal Turret Stabilizer — Host & Firmware Controls

Overview

This repo contains a minimal STM32F4 bare-metal firmware and small host utilities to control an RCWS gimbal's yaw and pitch ESCs. The firmware drives TIM1 PWM channels for ESC signals and can run a basic PID stabilization loop based on incoming IMU angle frames over UART.

Files of interest

- main.c — firmware entry. Initializes TIM1 and USART2, arms ESCs, and runs either manual command handling or PID stabilization (parses IMU frames).
- host_control.c — POSIX host program. Reads keyboard arrow keys and sends single-byte commands over a serial port to the board.
- host_control.py — (optional) Python version of the host keyboard bridge.
- Makefile, startup_stm32f411.s, linker.ld — build and link files for firmware.

Build & Flash (firmware)

Requires arm-none-eabi toolchain on PATH.

To build:

```bash
make
```

This produces `firmware.elf` and `firmware.bin`.

Flash using your programmer (example using `st-flash` or OpenOCD) — follow your usual toolchain.

Host programs (Linux)

C host (recommended):

Build:

```bash
gcc -o host_control host_control.c
```

Run:

```bash
./host_control /dev/ttyUSB0 [baud]
```

Python host (optional):

```bash
pip3 install pyserial
python3 host_control.py /dev/ttyUSB0
```

Controls (host -> board)

- Left arrow  : send `L` — rotate left (adjust yaw setpoint)
- Right arrow : send `R` — rotate right (adjust yaw setpoint)
- Up arrow    : send `U` — increase pitch setpoint
- Down arrow  : send `D` — decrease pitch setpoint
- Space / `s` : send `S` — stop / neutral (resets setpoints to zero)
- `q`         : quit host

Firmware serial protocol

The firmware accepts two kinds of serial input on `USART2`:

1) Single-character commands (from host):
   - `L` / `R` / `U` / `D` / `S` — adjust yaw/pitch setpoints or reset

2) IMU frames (ASCII text) for stabilization input:
   - Format: `I,<pitch>,<yaw>\n` (angles in degrees, floats)
   - Example: `I,1.25,-3.5\n`

When IMU frames are received the firmware updates measured angles and runs a PID loop (50Hz) to compute microsecond offsets to apply to the ESC PWM outputs.

ESC PWM mapping and safety

- Neutral PWM is set to 1500 µs by default.
- PID output is mapped with a configurable scale (in `main.c`) and clamped to the safe range 1000–2000 µs.
- Verify your ESC/motor wiring, and ensure propellers are removed during tuning.

Tuning

- PID gains in `main.c` are example values. Tune `kp`, `ki`, `kd` for both `pid_yaw` and `pid_pitch` carefully.
- Adjust `scale_us_per_deg` to map PID output (degrees) into PWM microseconds suitable for your actuator.

Notes & troubleshooting

- Ensure `MX_TIM1_Init()` configures both TIM1 channel 1 and channel 2 and that their GPIO pins are connected to ESC signal wires.
- The code uses `USART2` — ensure the board's UART2 TX/RX pins are connected to your USB-serial adapter and that the host uses the correct device (e.g., `/dev/ttyUSB0`).
- IntelliSense stubs are provided under `.vscode/stubs` to help editing; they are not used for building the firmware.

If you want, I can:
- Add a host tool mode that sends simulated IMU frames for tuning.
- Add build rules to `Makefile` to compile `host_control` as part of the workspace.

Autotune (optional)

You can enable a simple, built-in autotune routine in the firmware by compiling with `-DENABLE_AUTOTUNE`.
This adds a blocking autotune routine that performs short step tests on the yaw axis and chooses
a `Kp` candidate, then sets `Ki`/`Kd` using simple heuristics. To enable, add the define to your
compiler flags (example):

```bash
make CFLAGS+='-DENABLE_AUTOTUNE'
```

Usage: while the firmware is running and the IMU is streaming frames (`I,<pitch>,<yaw>\n`), send the
single character `T` from the host to trigger autotune. The autotune routine will move the gimbal —
remove propellers and secure the device before running.

Notes:
- The autotune here is intentionally conservative and basic. It requires the IMU frames to be provided
   over `USART2` during the tuning run so the firmware can observe step responses.
- Tuning results are heuristic (Kp sweep + simple Ki/Kd rules). Use autotune output only as a starting
   point and refine gains manually afterwards.

Stabilization math

The firmware implements a standard PID controller per axis (yaw, pitch). The continuous-time formulas are:

Error:
$$e(t)=\text{setpoint}(t)-\text{measurement}(t)$$

PID controller:
$$u(t)=K_p e(t) + K_i \int_0^t e(\tau)\,d\tau + K_d \frac{d e(t)}{dt}$$

Discrete implementation used in `main.c` (per loop at timestep $\Delta t$):

$$\text{integral} \mathrel{{+}{=}} e\cdot\Delta t$$
$$\text{derivative} = \frac{e- e_{\text{prev}}}{\Delta t}$$
$$u = K_p e + K_i\cdot\text{integral} + K_d\cdot\text{derivative}$$

Mapping to PWM microseconds:

$$\text{pwm}_{\mu s} = \text{neutral}_{\mu s} + s\cdot u$$

Where:
- $s$ is `scale_us_per_deg` (example: 2.0 \(\mu\)s/degree in the code)
- `neutral_us` is 1500 \(\mu\)s by default

The firmware clamps outputs to the safe ESC range: $[1000,2000]\ \mu s$.

Default parameters in code:
- `kp = 6.0`, `ki = 0.5`, `kd = 0.2` (for both yaw and pitch)
- `neutral_us = 1500` \(\mu\)s
- `scale_us_per_deg = 2.0` \(\mu\)s/degree
- PID loop rate: ~50 Hz (implemented with a 20 ms step)

Tuning notes:
- Remove propellers and secure payload before tuning.
- Start with `ki=0` and `kd=0`, increase `kp` until the system responds with reasonable speed but without oscillation.
- Add small `kd` to damp oscillations, then add `ki` to remove steady-state error.
- Adjust `scale_us_per_deg` so that expected angular commands map to safe PWM offsets.


