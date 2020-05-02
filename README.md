# DRGON-01 Ventilator
Ventilator Repository developed during the COVID-19 Pandemic in Mexico.

Current Status:
Initial version of the DRGON-01 Ventilator Software.
- Configured STM32F103C8:
    - Timers
        - TIM1: DC Motor PWM Command
        - TIM2: System Base Timer
        - TIM3: Encoder Pulse Reading
    - Buses
        - I2C Bus 1: LCD Display Communication
- Created Encoder API (encoder_api.c and encoder_api.h)
- Created DC Motor API (dc_motor_api.c and dc_motor_api.h)
- Created LCD Display API (lcd_display_api.c and lcd_display_api.h)
- Created the following tasks on main:
    - Master: Used to manage the different tasks such as initialization, calibration, that would only run once.
    - DC Motor: Test routine for the DC motor control.
    - LCD Update: Periodically updates the RPM and PWM values.
    - Encoder: Periodically calculates the RPMs of the encoder.


TODO: Add info and photos.