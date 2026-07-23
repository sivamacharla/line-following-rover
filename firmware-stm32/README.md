# STM32 Nucleo-F401RE Port

Port of the line-following rover firmware from Arduino to the NUCLEO-F401RE
(STM32F401RE, Cortex-M4 @ up to 84MHz), built with STM32CubeIDE + HAL.

> **This code has not been compiled or flashed** — this environment has no
> ARM toolchain / STM32CubeIDE installed, so unlike the Arduino/Python code
> in this repo, it hasn't been build-verified. It follows standard HAL
> patterns carefully, but budget time to fix compiler errors on first build,
> and treat the logic (control loop, sensor fusion) as the reviewed part —
> the peripheral init boilerplate is the part most likely to need tweaking
> once CubeMX generates it for your exact project.

## Why HAL + CubeMX instead of a plain source drop

STM32 projects need generated startup files, a linker script, and HAL driver
sources that CubeMX produces correctly for your exact chip/pin configuration.
Hand-writing those reliably (without a toolchain here to verify) is a bad
trade — so the workflow is: **you generate the project skeleton with CubeMX
using the pin table below, then replace `Core/Src/main.c` / `Core/Inc/main.h`
with the versions in this folder** (merging into the `USER CODE` sections CubeMX
leaves for you).

## 1. Create the project in STM32CubeIDE

1. `File > New > STM32 Project` → select board **NUCLEO-F401RE** → Next → name it (e.g. `rover-stm32`) → Finish. Accept the default "Initialize all peripherals with their default Mode" = **No**.
2. In the `.ioc` pinout view, configure each pin per the table below (click the pin, pick the mode from the dropdown).
3. Configure each peripheral in the left panel (Analog / Timers / Connectivity) per the settings below.
4. `Clock Configuration` tab → set **HCLK to 84 MHz** (CubeIDE will auto-resolve the PLL from HSI; accept its suggested values).
5. `Project > Generate Code`.
6. Replace the generated `Core/Src/main.c` and `Core/Inc/main.h` with the files in this folder (keep CubeMX's `USER CODE BEGIN/END` init calls in `main()` — drop the application logic from `main.c` here into the matching `USER CODE` sections, or wholesale-replace if the generated init function bodies match).

## 2. Pin configuration

| Signal | Nucleo pin | MCU pin | Mode |
|---|---|---|---|
| IR sensor 0 | A0 | PA0 | ADC1_IN0 |
| IR sensor 1 | A1 | PA1 | ADC1_IN1 |
| IR sensor 2 | A2 | PA4 | ADC1_IN4 |
| IR sensor 3 | A3 | PB0 | ADC1_IN8 |
| IR sensor 4 | A4 | PC1 | ADC1_IN11 |
| Ultrasonic TRIG | D8 | PA9 | GPIO_Output |
| Ultrasonic ECHO | D9 | PC7 | GPIO_Input |
| Left motor PWM (ENA) | D12 | PA6 | TIM3_CH1 |
| Left motor IN1 | D10 | PB6 | GPIO_Output |
| Left motor IN2 | D5 | PB4 | GPIO_Output |
| Right motor PWM (ENB) | D11 | PA7 | TIM3_CH2 |
| Right motor IN3 | D6 | PB10 | GPIO_Output |
| Right motor IN4 | D4 | PB5 | GPIO_Output |
| Telemetry UART TX | D1 | PA2 | USART2_TX |
| Telemetry UART RX | D0 | PA3 | USART2_RX |

USART2 is the same UART wired to the ST-LINK's virtual COM port, so telemetry
comes out over the same USB cable used to flash the board — no extra
USB-serial adapter needed. Open it in the dashboard's **Connect Arduino
(Live)** button (Web Serial works with any USB-serial device, not just
Arduino) or any serial monitor at 115200 baud.

## 3. Peripheral settings

- **ADC1**: Independent mode, IN0/IN1/IN4/IN8/IN11 enabled, 12-bit resolution, no continuous/scan conversion needed (the code reads one channel at a time via `HAL_ADC_ConfigChannel` + single conversion — simpler and more predictable than DMA scan mode for 5 low-rate channels).
- **TIM2**: General purpose, no output channels needed — used purely as a free-running microsecond counter for timing the ultrasonic echo pulse (equivalent to Arduino's `pulseIn`). Prescaler `83`, Counter Period `0xFFFFFFFF` (32-bit). This assumes TIM2's clock is 84MHz (APB1 timer clock is 2x PCLK1 when the APB1 prescaler isn't 1, which is the default CubeMX gives you at 84MHz HCLK) — double check the `Clock Configuration` tab shows APB1 Timer clocks = 84MHz, and adjust the prescaler if yours differs.
- **TIM3**: PWM Generation CH1 + CH2, Prescaler `83`, Counter Period `999` (→ 1kHz PWM, duty cycle range 0–999). Same APB1-clock caveat as TIM2.
- **USART2**: Asynchronous, 115200-8-N-1.
- **SYS**: Debug = Serial Wire (default).

## 4. Build and flash

`Project > Build`, then `Run > Debug` (or the flash icon) with the board connected via USB. STM32CubeIDE flashes through the onboard ST-LINK automatically.

## 5. Differences from the Arduino version

- ADC is 12-bit (0–4095) here vs. Arduino's 10-bit (0–1023). `readIRSensor()` normalizes to a 0.0–1.0 float activation per sensor (matching the Python simulation's convention), so `SENSOR_WEIGHTS` and the "line lost" threshold are resolution-independent.
- PWM duty cycle range is 0–999 (TIM3 ARR) instead of Arduino's 0–255 `analogWrite` scale — `BASE_SPEED`/`MAX_SPEED` are defined in that range.
- Ultrasonic pulse timing uses TIM2 as a free-running counter polled in a busy-loop, mirroring `pulseIn()`'s approach exactly rather than using timer input-capture interrupts — simpler to reason about and close enough at this control-loop rate (50Hz).
- Loop timing uses `HAL_GetTick()` (1ms SysTick) the same way the Arduino version uses `millis()`.

## 6. Same tuning gains

`KP`/`KI`/`KD`/`D_FILTER_ALPHA`/`PID_OUTPUT_LIMIT` in `main.c` match the values
validated in [`simulation/rover_sim.py`](../simulation/rover_sim.py) (Kp=34,
Ki=0.6, Kd=9.5) — re-tune on real hardware, but this is a reasonable start.
