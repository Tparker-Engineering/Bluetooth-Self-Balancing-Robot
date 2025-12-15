# Bluetooth-Self-Balancing-Robot
Firmware for a two-wheel self-balancing robot that uses an MPU-6050 IMU + wheel encoders and accepts basic motion + tuning commands over a Bluetooth serial link (UART).

This project runs a fast balance loop (timer interrupt) and combines:
- **Angle estimation** using a Kalman filter (gyro + accelerometer fusion).  
- **Balance control** using PD feedback.
- **Speed control** using an outer PI loop from encoder ticks.
- **Turn control** using a yaw-rate term mixed into left/right motor commands.

---

## What’s in here

### Firmware modules
- `BtBalanceRobot.ino` — main firmware (init, timer ISR control loop, command handling, telemetry)
- `imu.c` — MPU-6050 driver over AVR TWI/I²C (initialization + burst reads)  
- `kalmanFilter.c` — Kalman predict/update for pitch angle estimation  
- `motor.c` — motor driver + PWM setup + encoder ISR counting  
- `gpio.c` — small GPIO layer for ATmega328p/Arduino Nano pin mapping  
- `bluetooth.c` — UART init + basic send/receive helpers (used by Bluetooth serial)

---

## Hardware assumptions (at a high level)

- **MCU:** ATmega328p-class board (ex: Arduino Nano)
- **IMU:** MPU-6050 over I²C/TWI  
- **Motor driver:** dual H-bridge style interface (DIR + PWM + STBY)
- **Encoders:** at least one channel per wheel (counted via interrupts)

---

## Bluetooth / UART command interface

### Motion commands (single character)
- `F` — forward
- `B` — backward
- `S` — stop (keep balancing)
- `L` — turn left
- `R` — turn right
- `G` — print current gains
- `T` — toggle telemetry on/off

### Gain tuning commands (two letters + value)
Send ASCII like:

- `PB <value>` — set balance proportional gain  
- `DB <value>` — set balance derivative gain  
- `PS <value>` — set speed proportional gain  
- `IS <value>` — set speed integral gain  
- `PT <value>` — set turn proportional gain

Gains are printed back over UART after updates.

---

## Telemetry

When telemetry is enabled, the firmware periodically prints a compact status line over UART (pitch, control output, filtered speed). This is useful for tuning and debugging over Bluetooth serial.
