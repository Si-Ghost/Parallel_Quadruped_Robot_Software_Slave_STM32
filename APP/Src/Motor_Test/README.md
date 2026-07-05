# Left Front Motor Spin Test

This folder contains a dedicated low-risk STM32 test mode for the left-front
RS485 motor bus:

- UART2 TX: PA2
- UART2 RX: PA3
- RS485 DE: PA4
- Expected motor IDs on this bus: 0 and 1
- Debug console: UART1 at 115200 baud

The test firmware does not start the ESP32 link or the full robot control loop.
It only talks to the left-front motor bus.

## Build

With CMake:

```powershell
cmake --preset Debug -DENABLE_MOTOR_SPIN_TEST=ON
cmake --build --preset Debug
```

To return to the normal firmware:

```powershell
cmake --preset Debug -DENABLE_MOTOR_SPIN_TEST=OFF
cmake --build --preset Debug
```

## Serial Commands

After flashing the test build, open UART1 at 115200 baud.

- `p`: ping ID0 and ID1 with zero command
- `a`: spin ID0 at +0.5 rad/s
- `z`: spin ID0 at -0.5 rad/s
- `k`: spin ID1 at +0.5 rad/s
- `m`: spin ID1 at -0.5 rad/s
- `s`: stop both motors

No motor spins until `a`, `z`, `k`, or `m` is received.

## Expected Result

If the bus is healthy, `p` should print `OK` for each connected motor and show
feedback fields such as `fbk_id`, `pos`, `W`, `T`, `temp`, and `err`.

If it prints `TIMEOUT`, check motor power, RS485 A/B wiring, UART2 pins, and
the PA4 direction-control signal. If it prints `UART/FRAME_ERROR`, check baud
rate, frame length, signal quality, and protocol compatibility.
