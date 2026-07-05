# Motor Spin Minimal

This is a standalone STM32 motor-bus test project. It intentionally does not
start ESP32 communication, the web UI, the full quadruped control loop, or the
multi-leg scan logic.

The goal is to isolate one hardware path:

- Left-front motor bus only
- USART2 TX: PA2
- USART2 RX: PA3
- RS485 direction control: PA4
- Debug console: USART1 at 115200 baud
- Motor baud rate: 4 Mbps
- Expected motor IDs on this bus: 0 and 1

Using UART1 for debug is intentional here: it keeps the ESP32 out of the test
path. If this project cannot ping the motor, the problem is below the ESP32
layer: motor power, RS485 wiring, direction-control timing, UART2 waveform, ID,
or protocol/baud compatibility.

## Build

From this folder:

```powershell
cmake -S . -B build -G Ninja "-DCMAKE_TOOLCHAIN_FILE=D:/Code/Parallel_Quadruped_Robot/Software/STM32/Motor_Spin_Minimal/cmake/gcc-arm-none-eabi.cmake"
cmake --build build
```

The output ELF is:

```text
build/Motor_Spin_Minimal.elf
```

## Test Commands

Open USART1 at 115200 baud after flashing.

- `p`: ping ID0 and ID1 with zero command
- `a`: spin ID0 at +0.5 rad/s
- `z`: spin ID0 at -0.5 rad/s
- `k`: spin ID1 at +0.5 rad/s
- `m`: spin ID1 at -0.5 rad/s
- `s`: stop both motors

The firmware does not spin any motor until `a`, `z`, `k`, or `m` is received.

## Interpreting Results

`OK` means the STM32 sent a command and received a valid motor feedback frame.

`TIMEOUT` means STM32 received no response. Check:

- motor power
- common ground
- RS485 A/B wiring
- PA4 DE signal
- PA2 UART waveform
- motor ID

`UART/FRAME_ERROR` means some response was received, but the frame was invalid
or incomplete. Check baud rate, signal quality, protocol compatibility, and
whether the RS485 direction pin switches back to receive soon enough.
