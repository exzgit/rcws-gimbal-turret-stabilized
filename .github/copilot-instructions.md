# Copilot / AI Agent Instructions for RCWSGimbalTurretStabilizer

Purpose: help AI coding agents become productive quickly in this small STM32 embedded repo.

- **Big picture:** this is a bare-metal STM32F4 project that builds a single `firmware` binary. The build compiles `startup_stm32f411.s` + `main.c` and links with `linker.ld` to produce `firmware.elf` and `firmware.bin` (see Makefile). The project uses the STM32 HAL API (`#include "stm32f4xx_hal.h"`) and drives TIM1 PWM for an ESC/motor.

- **Key files:**
  - `Makefile` — build rules and important toolchain variables (`CC=arm-none-eabi-gcc`, `OBJCOPY=arm-none-eabi-objcopy`).
  - `startup_stm32f411.s` — vector table and `Reset_Handler` that branches to `main`.
  - `linker.ld` — memory layout (FLASH 512K, SRAM 128K) and sections mapping.
  - `main.c` and `mains.c` — application entry; Note: `Makefile` references `main.c`. `mains.c` appears to be a near-duplicate; do not change `Makefile` unless intentionally switching mains.

- **Build & artifacts:**
  - Run `make` (requires `arm-none-eabi` toolchain on PATH). Output: `firmware.elf` and `firmware.bin`.
  - Compile command template from `Makefile`: `$(CC) $(CFLAGS) startup_stm32f411.s main.c $(LDFLAGS) -o firmware.elf` then `objcopy -O binary` to create `.bin`.

- **Discoverable conventions / patterns:**
  - HAL is used (e.g. `HAL_Init()`, `MX_TIM1_Init()`, `HAL_TIM_PWM_Start`). Look for HAL handles like `TIM_HandleTypeDef htim1` in `main.c`.
  - PWM & ESC handling: `set_motor_speed_us(uint16_t us)` is declared and used in `main.c` / `mains.c` but its definition is not present in this repo — search workspace for its implementation or add a PWM driver module if missing.
  - TIM1-specific enable: code calls `__HAL_TIM_MOE_ENABLE(&htim1)` — treat TIM1 main output enable as required hardware behavior.

- **Important low-level details to respect:**
  - `linker.ld` defines memory ranges (FLASH 0x08000000, SRAM 0x20000000, sizes 512K/128K). Do not change these addresses without device datasheet verification.
  - `startup_stm32f411.s` sets `_estack = 0x2002000` (comment says 128KB SRAM). This value looks incorrect vs. `linker.ld` (expected top of SRAM 0x20020000). Verify `_estack` before editing; incorrect stack pointer will prevent boot.

- **What an AI agent should do first when modifying code:**
  1. Confirm which `main` file is active (`Makefile` uses `main.c`). If you need to swap to `mains.c`, also update `Makefile`.
 2. Search for `set_motor_speed_us` implementation; if missing, implement a small PWM helper that converts microseconds to TIM compare values and updates `htim1` CCR register.
 3. When changing low-level startup or linker settings, cross-check `startup_stm32f411.s` `_estack` value with `linker.ld` memory ORIGIN+LENGTH.

- **Debug / flash hints (repo-observable):**
  - The repo produces `firmware.bin` suitable for flashing with an external tool (ST-Link, OpenOCD, etc.). The project does not include a flashing script — use your standard toolchain commands.
  - For debugging, `firmware.elf` can be used with `arm-none-eabi-gdb` plus an adapter (OpenOCD / ST-Link).

- **Quick examples from this repo:**
  - Entry/boot: see `startup_stm32f411.s` and `linker.ld`.
  - PWM start: `HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);` in `main.c`.
  - ESC arming sequence: `set_motor_speed_us(1000); HAL_Delay(3000);` in `main.c`/`mains.c`.

- **When adding files or libraries:**
  - If adding external HAL sources or vendor drivers, update `Makefile` to add include paths (`-I`) and additional source files in the compile rule.

- **What NOT to change lightly:**
  - `linker.ld` memory layout, interrupt vector defaults, or the `Reset_Handler` behavior in `startup_stm32f411.s` without validating on hardware or via a debugger.

If anything above is unclear or you want the agent to implement a missing PWM driver or correct the `_estack` value, tell me which change to make and I will proceed.
