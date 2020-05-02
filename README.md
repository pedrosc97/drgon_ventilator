# DRGON Ventilator

### Basic Firmware Release v0.1
- **DC Motor API**: fully configurable firmware library for interaction with DC Motors.
- **Encoder API**: fully configurable firmware library for interaction with Encoders.
- **LCD Display API**: firmware library for interaction with I2C LCD Display.
- **Potentiometer API**: firmware library for configuration and conversion of potentiometers for parameter input.
- **Ventilator API**: library for the configuration and parameter conversion and recording of the ventilator values, such as I:E Ratio, Respiration Periods, etc.
- **Semi-Open Loop Respiration Cycle Routine**: basic respiration cycle, taking into account the different ventilator parameters and with a safety feature for the overshooting of the angle.

### Bugfixes
- Fixed a bug where the LCD Display would stop updating after a certain amount of time.
- Fixed a bug where the interrupts were being triggered too early, thus messing up with the initialization of some peripherals.
