# XRMotor

Common motor interfaces for XRobot modules.

Contents:
- `brushed_motor_driver.hpp`: abstract brushed motor driver interface
- `motor.hpp`: generic motor module template that composes a driver and a board encoder

This repository is platform-agnostic. Board-specific PWM/GPIO/QEI implementations stay in the
consumer project.
